/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Universidad de Cantabria
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Pablo Garrido Ortiz <pgarrido@tlmat.unican.es>
 * 		   Eduardo Rodríguez Maza <eduardo.rodriguez@alumnos.unican.es>
 * 		   David Gómez Fernández <dgomez@tlmat.unican.es>
 *		   Ramón Agüero Calvo <ramon@tlmat.unican.es>
 */

#include "intra-flow-network-coding-protocol.h"

#include "ns3/config.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/llc-snap-header.h"
#include "ns3/ipv4-header.h"
#include "ns3/udp-header.h"
#include "ns3/wifi-net-device.h"
#include "ns3/regular-wifi-mac.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/hash-id.h"

#include <ctime>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <cstdlib>
#include <iostream>
#include <bitset>
#include <vector>
#include <cmath>
//#include <random>

#include <m4rie/m4rie.h>

using namespace ns3;
using namespace std;
using namespace itpp;

#define RATEPROBE 10


NS_LOG_COMPONENT_DEFINE ("IntraFlowNetworkCodingProtocol");
NS_OBJECT_ENSURE_REGISTERED (IntraFlowNetworkCodingProtocol);

/* see http://www.iana.org/assignments/protocol-numbers */
const uint8_t IntraFlowNetworkCodingProtocol::PROT_NUMBER = 100;

float timeval_diff(struct timeval *a, struct timeval *b) // Calculates time in seconds
{
	return	(float) (a->tv_sec + (float) a->tv_usec / 1000000) -
			(float) (b->tv_sec + (float) b->tv_usec / 1000000);
}

IntraFlowNetworkCodingBufferItem::IntraFlowNetworkCodingBufferItem ()
{
	sourcePort=0;
	destinationPort=0;
}

IntraFlowNetworkCodingStatistics::IntraFlowNetworkCodingStatistics():  txNumber(0), rxNumber(0), downNumber(0), upNumber(0)
{
	timestamp.clear();
	rankTime.clear();
	inverseTime.clear();

	downNumber = 0;
	upNumber = 0;
	packetLength = 0;
	rxNumber  = 0;
	txNumber = 0;
}

IntraFlowNetworkCodingBufferItem::IntraFlowNetworkCodingBufferItem (Ptr<Packet> packet, Ipv4Address source, Ipv4Address destination, u_int16_t sourcePort, u_int16_t destinationPort):
packet(packet),
source(source),
destination(destination),
sourcePort(sourcePort),
destinationPort(destinationPort)
{
}

IntraFlowNetworkCodingMapParameters::IntraFlowNetworkCodingMapParameters ()
{
	m_k=0;
	m_rank=0;
	m_fragmentNumber=0;
	m_forwardingNode = false;
	m_receivedPackets = 0;
}

IntraFlowNetworkCodingMapParameters::~IntraFlowNetworkCodingMapParameters ()
{
	m_txBuffer.clear();
	m_rxBuffer.clear();
}

TypeId IntraFlowNetworkCodingProtocol::GetTypeId (void)
{

	static TypeId tid = TypeId ("ns3::IntraFlowNetworkCodingProtocol")
	.SetParent<NetworkCodingL4Protocol> ()
	.AddConstructor<IntraFlowNetworkCodingProtocol> ()
	.AddAttribute ("Q",
				"Size of the finite field GF(2^Q)",
				UintegerValue (1),
				MakeUintegerAccessor (&IntraFlowNetworkCodingProtocol::m_q),
				MakeUintegerChecker<u_int8_t> ())
	.AddAttribute ("K",
				"Fragment Size",
				UintegerValue (8),
				MakeUintegerAccessor (&IntraFlowNetworkCodingProtocol::m_k),
				MakeUintegerChecker<u_int16_t> ())
	.AddAttribute ("Recoding",
				"Differentiate between a Network coding and a Source coding solution",
				BooleanValue (false),
				MakeBooleanAccessor (&IntraFlowNetworkCodingProtocol::m_recode),
				MakeBooleanChecker ())
	.AddAttribute ("Systematic",
				"The source sends the first K packets not encode",
				BooleanValue (false),
				MakeBooleanAccessor (&IntraFlowNetworkCodingProtocol::m_systematic),
				MakeBooleanChecker ())
	.AddAttribute ("OverHearing",
				"All the nodes that overhear a transmission participate in the forwarding process",
				BooleanValue (false),
				MakeBooleanAccessor (&IntraFlowNetworkCodingProtocol::m_overhearing),
				MakeBooleanChecker ())
	.AddAttribute("StartRecoding",
				  "When the intermediate node has this number of packets, start to do recoding",
				  UintegerValue (0),
				  MakeUintegerAccessor (&IntraFlowNetworkCodingProtocol::m_startRecoding),
				  MakeUintegerChecker<uint16_t> ())
	.AddAttribute ("BufferTimeout",
				"Time during which the protocol will wait until the buffer has at least K packets",
				TimeValue (MilliSeconds(1000)),
				MakeTimeAccessor (&IntraFlowNetworkCodingProtocol::m_bufferTimeout),
				MakeTimeChecker());
	return tid;
}

IntraFlowNetworkCodingProtocol::IntraFlowNetworkCodingProtocol()
{
	NS_LOG_FUNCTION (this);

	m_randomGenerator = CreateObject<UniformRandomVariable>();

	//Statistics

	m_stats.source = false;
	m_stats.forwarder = true;
	m_stats.txBytes = 0;
	m_stats.txPackets = 0;
	m_stats.rxBytes = 0;
	m_stats.rxPackets = 0;
	m_stats.rxDiscacerdedPackets = 0;
	m_stats.packetLenght = 0;

}

IntraFlowNetworkCodingProtocol::~IntraFlowNetworkCodingProtocol()
{
	NS_LOG_FUNCTION_NOARGS();
	m_mapParameters.clear();

	m_stats.txTimestamp.clear();
	m_stats.rxTimestamp.clear();
	//gf2e_free(m_GF);
}

void IntraFlowNetworkCodingProtocol::NotifyNewAggregate()
{
	NS_LOG_FUNCTION_NOARGS();

	if (m_node == 0)
	{
		Ptr<Node> node = this->GetObject<Node> ();
		m_node = this->GetObject<Node> ();

		if (node != 0)
		{
			//Connection with IP layer
			// Downstream: IntraFlowNetworkCodingProtocol::DownTargetCallback --> Ipv4L3Protocol::Send
			Ptr<Ipv4> ipv4 = this->GetObject<Ipv4> ();
			if (ipv4 != 0)
			{
				this->SetNode (node);
				ipv4->Insert (this);
				this->SetDownTarget (MakeCallback (&Ipv4::Send, ipv4));
			}

			//Connect to IpV4L3Protocol::IpForward
			Ptr<Ipv4L3Protocol> ipv4L3 = this->GetObject <Ipv4L3Protocol > ();
			if (ipv4L3)
			{
				ipv4L3->SetIpForwardCallback(MakeCallback(&IntraFlowNetworkCodingProtocol::ParseForwardingReception, this));
			}

			//Connection with UDP layer
			//	Downstream: UdpL4Protocol::Send --> IntraFlowNetworkCodingProtocol::ReceiveFromUpperLayer
			//  Upstream:   IntraFlowNetworkCodingProtocol::ForwardUp --> UdpL4Protocol::Receive
			if (Ptr<UdpL4Protocol> udp = node->GetObject <UdpL4Protocol> ())
			{
				udp->SetDownTarget (MakeCallback (&IntraFlowNetworkCodingProtocol::ReceiveFromUpperLayer, this));
				this->SetUpUdpTarget (MakeCallback(&UdpL4Protocol::ReceiveIpv4,udp));
			}
			//Connect to the WifiNetMacQueue and YansWifiPhy objects
			for (u_int8_t i=0; i < node->GetNDevices(); i++)
			{
				Ptr<WifiNetDevice> wifi = node->GetDevice(i)->GetObject<WifiNetDevice>();
				if (wifi)
				{
					wifi->SetPromiscReceiveCallback (MakeCallback(&NetworkCodingL4Protocol::ReceivePromiscuous, this));
					SetFlushWifiBufferCallback (MakeCallback (&WifiMacQueue::Flush,  DynamicCast<RegularWifiMac>(node->GetDevice(i)->GetObject<WifiNetDevice>()->GetMac())->GetDcaTxopPub()->GetQueue()));
					SetSelectiveFlushWifiBufferCallback (MakeCallback (&WifiMacQueue::SelectiveFlush,  DynamicCast<RegularWifiMac>(node->GetDevice(i)->GetObject<WifiNetDevice>()->GetMac())->GetDcaTxopPub()->GetQueue()));
				}
			}

			ostringstream os;
			os << (int) node->GetId();

			//Connect to the WifiPhy traced callback
			Config::ConnectWithoutContext ("/NodeList/" +  os.str() +"/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
					MakeCallback (&IntraFlowNetworkCodingProtocol::WifiBufferEvent, this));

		}
	}

	//Statistics
	m_stats.q = m_q;
	m_stats.k = m_k;

	Object::NotifyNewAggregate ();
}

void IntraFlowNetworkCodingProtocol::ReceiveFromUpperLayer (Ptr<Packet> packet, Ipv4Address source, Ipv4Address destination, uint8_t protocol, Ptr<Ipv4Route> route)
{
	NS_LOG_FUNCTION (Simulator::Now().GetSeconds() << this );

	UdpHeader udpHeader;
	u_int16_t flowId;
	Ptr <IntraFlowNetworkCodingMapParameters> mapParameters;
	IntraFlowMapIterator it;

	if (packet-> GetSize() > 250) 	// All datagrams < 250 bytes will be immediately delivered downwards
	{
		packet->PeekHeader (udpHeader);
		flowId = HashID (source, destination, udpHeader.GetSourcePort(), udpHeader.GetDestinationPort());
		it = m_mapParameters.find(flowId);
		if (it == m_mapParameters.end())  //Is the first packet of that flow. Initialize
		{
			Ptr<IntraFlowNetworkCodingMapParameters> aux = CreateObject<IntraFlowNetworkCodingMapParameters> ();
			aux->m_k = m_k;
			aux->m_fragmentNumber = 0;
			aux->m_forwardingNode = false;
			aux->m_rank = 0;
			aux->m_packetSize = packet->GetSize();

			m_mapParameters.insert(pair<u_int16_t, Ptr<IntraFlowNetworkCodingMapParameters> >(make_pair(flowId, aux)));

			//Statistics
			m_stats.forwarder = false;
			m_stats.source = true;
			m_stats.recode = m_recode;
		}

		it = m_mapParameters.find(flowId);
		mapParameters = it->second;
		mapParameters->m_txBuffer.push_back(IntraFlowNetworkCodingBufferItem(packet, source, destination, udpHeader.GetSourcePort(), udpHeader.GetDestinationPort()));
		mapParameters->m_receivedPackets++;

		if (mapParameters->m_receivedPackets < mapParameters->m_k)  // In case there are not enough packets to encode
		{
			if (!m_reduceBufferEvent.IsRunning())  // Used for the timer of ReduceBuffer() method which is created when there are less than k packets left
			{
				m_reduceBufferEvent = Simulator::Schedule (m_bufferTimeout, &IntraFlowNetworkCodingProtocol::ReduceBuffer, this, flowId);
			}
		}
		else
		{
				Encode(flowId);
		}

		//Statistics
		m_stats.rxPackets ++;
		m_stats.rxBytes += packet->GetSize();
		m_stats.rxTimestamp.push_back(Simulator::Now().GetSeconds());
	}
	else	// All datagrams < 250 bytes will be immediately delivered downwards
	{
		IpL4Protocol::DownTargetCallback downTarget = GetDownTarget();
		downTarget (packet, source, destination, UdpL4Protocol::PROT_NUMBER, 0);
	}

}

void IntraFlowNetworkCodingProtocol::SendToLowerLayer(Ptr<Ipv4Route> rtentry, Ptr<const Packet> packet, const Ipv4Header &header)
{
	NS_LOG_FUNCTION_NOARGS();

	Ptr<Packet> sentPacket = packet->Copy();
	Ptr<Ipv4L3Protocol> ipv4 = m_node->GetObject <Ipv4L3Protocol > ();
	if (ipv4)
		ipv4->SendRealOutHook(rtentry, sentPacket, header);

}

void IntraFlowNetworkCodingProtocol::DoDispose (void)
{
	NS_LOG_FUNCTION_NOARGS();
}

int IntraFlowNetworkCodingProtocol::GetProtocolNumber (void) const
{
	NS_LOG_FUNCTION_NOARGS();
	return PROT_NUMBER;
}

void IntraFlowNetworkCodingProtocol::Encode (u_int16_t flowId)
{
	//Generate Coded Packet
	NS_LOG_FUNCTION (m_node->GetId()  << Simulator::Now().GetSeconds() << this << flowId );

	IntraFlowMapIterator it;
	Ptr <IntraFlowNetworkCodingMapParameters> mapParameters;
	Ptr<Packet> codedPacket;
	IntraFlowNetworkCodingHeader ncHeader;
	std::vector<u_int8_t> randomVector;

	it=m_mapParameters.find(flowId);
	mapParameters=it->second;

	if (mapParameters->m_receivedPackets >= mapParameters->m_k )
	{
 		if (m_reduceBufferEvent.IsRunning())
		{
			m_reduceBufferEvent.Cancel();
		}

 		if (m_systematic && mapParameters->m_rank < mapParameters->m_k)
 		{
 			for (u_int16_t i=0 ; i < (mapParameters->m_k) ; i++ )
 			{
 				randomVector.push_back(0);
 			}
 			randomVector[mapParameters->m_rank] = 1;

 			mapParameters->m_rank ++;
 		}
 		else
 		{
 			GenerateRandomVector(mapParameters->m_k, randomVector);
 			mapParameters->m_rank ++;
 		}

		codedPacket = Create <Packet> (mapParameters->m_packetSize); // Packet creation with the buffer packet size
		m_stats.packetLenght = codedPacket->GetSize();

		//Generate NC header
		ncHeader.SetK (mapParameters->m_k);
		ncHeader.SetQ (m_q);
		ncHeader.SetNfrag (mapParameters->m_fragmentNumber);
		ncHeader.SetSourcePort (mapParameters->m_txBuffer.back().sourcePort);
		ncHeader.SetDestinationPort (mapParameters->m_txBuffer[0].destinationPort);
		ncHeader.SetVector(randomVector);

		randomVector.clear (); // Erasure of the random vector
		codedPacket->AddHeader (ncHeader); // Adding the MORE header to the packet
//		if (!m_ncCallback.IsNull())  //Statistics
//		{
//			m_ncCallback(codedPacket, 0, m_node->GetId(),mapParameters->m_txBuffer[0].source,mapParameters->m_txBuffer[0].destination);
//		}

		//Down the packet lower layer
		IpL4Protocol::DownTargetCallback downTarget = GetDownTarget();
		downTarget (codedPacket, mapParameters->m_txBuffer[0].source,mapParameters->m_txBuffer[0].destination, IntraFlowNetworkCodingProtocol::PROT_NUMBER, 0); // The node 0 is taken because there are only 2 nodes
	}

}

void IntraFlowNetworkCodingProtocol::Recode (u_int16_t flowId, bool last)
{
	NS_LOG_FUNCTION (Simulator::Now().GetSeconds() << this );

	//Create a Re-coded Packet
	IntraFlowMapIterator it;
	Ptr <IntraFlowNetworkCodingMapParameters> mapParameters;
	Ptr<Packet> codedPacket;
	IntraFlowNetworkCodingHeader ncHeader;

	std::vector<u_int8_t> randomVector;
	std::vector<u_int8_t> recodedVector;
	bool exit = false;

	if (m_reduceBufferEvent.IsRunning())
	{
		m_reduceBufferEvent.Cancel();
	}

	it=m_mapParameters.find (flowId);
	mapParameters=it->second;
	std::vector <u_int8_t> zeros (mapParameters->m_k, 0);
	if(mapParameters->m_receivedPackets > 0)	// This is because sometimes the MORE buffer is empty
	{
		codedPacket = Create <Packet> (mapParameters->m_packetSize);

		while(!exit)  //Not send Zero Coefficients vector
		{
			if ( mapParameters->m_rank < m_startRecoding)
			{
				for (u_int16_t i=0 ; i < (mapParameters->m_k) ; i++ )
				{
					randomVector.push_back(0);
				}
				randomVector[mapParameters->m_rank-1] = 1;
			}
			else{
				GenerateRandomVector(mapParameters->m_k, randomVector);
			}

			if(m_q==1)
			{
				itpp::bvec recodedVectorItpp;
				itpp::bvec randomVectorItpp;

				for(u_int8_t i = 0; i < randomVector.size(); i++)
				{
					int valuen= randomVector [i];
					randomVectorItpp.ins (i,valuen);
				}

				recodedVectorItpp = mapParameters->m_vectorMatrix.transpose() * randomVectorItpp; // We use the transpose because itpp only has the operator to do "matrix*vector" and not "vector*matrix"
				for(u_int8_t i = 0; i < randomVector.size(); i++)
				{
					int value = (int)recodedVectorItpp [i];
					recodedVector.push_back(value);
				}

				recodedVectorItpp.clear();
				randomVectorItpp.clear();
			}
			else
			{
				mzed_t *randomVectorGF = mzed_init(m_GF, (int) mapParameters->m_k, (int) mapParameters->m_k);
				mzed_t *recodedVectorGF = mzed_init(m_GF, (int) mapParameters->m_k, (int) mapParameters->m_k);
				for (int iter=0; iter < mapParameters->m_k; iter++)
				{
					uint64_t aux = randomVector[iter];
					mzed_write_elem(randomVectorGF,0,iter,aux);
				}

				mzed_mul_newton_john(recodedVectorGF,randomVectorGF,mapParameters->m_vectorMatrixGf);

				for(int i = 0; i < (int) randomVector.size(); i++)
				{
					int value = (uint8_t) mzed_read_elem(recodedVectorGF,0,i);
					recodedVector.push_back (value);
				}

				mzed_free(randomVectorGF);
				mzed_free(recodedVectorGF);
			}
			if (recodedVector == zeros)  //We do not send vectors full of zeros
			{
				randomVector.clear ();
				recodedVector.clear();
			}
			else
			{
				exit = true;
			}
		}

		//Declaration NC Header
		ncHeader.SetK (mapParameters->m_k);
		ncHeader.SetNfrag (mapParameters->m_fragmentNumber);
		ncHeader.SetQ(m_q);
		ncHeader.SetSourcePort (mapParameters->m_txBuffer[0].sourcePort);
		ncHeader.SetDestinationPort (mapParameters->m_txBuffer[0].destinationPort);
		ncHeader.SetTx(0);
		ncHeader.SetVector(recodedVector);

		codedPacket->AddHeader (ncHeader); // Adding the MORE header to the packet

//		if (!m_ncCallback.IsNull())
//		{
//			m_ncCallback(codedPacket, 7, m_node->GetId(),mapParameters->m_txBuffer[0].source,mapParameters->m_txBuffer[0].destination);
//		}

		IpL4Protocol::DownTargetCallback downTarget = GetDownTarget();
		downTarget (codedPacket, mapParameters->m_txBuffer[0].source,mapParameters->m_txBuffer[0].destination, IntraFlowNetworkCodingProtocol::PROT_NUMBER, 0);

		randomVector.clear (); //Clean random vectors
		recodedVector.clear ();
	}
}

void IntraFlowNetworkCodingProtocol::Decode(Ipv4Header const &header, Ptr<Ipv4Interface> incomingInterface, u_int16_t flowId)
{
	NS_LOG_FUNCTION (m_node->GetId() << Simulator::Now().GetSeconds() << this );

	struct timeval startTime, endTime;
	IntraFlowNetworkCodingBufferItem item;

	u_int16_t sourcePort=0;
	u_int16_t destinationPort=0;

	IntraFlowMapIterator it;
	Ptr <IntraFlowNetworkCodingMapParameters> mapParameters;

	it = m_mapParameters.find(flowId);
	mapParameters = it->second;

	if(m_q==1)
	{
		gettimeofday(&startTime, NULL);
		GF2mat inverse = mapParameters->m_vectorMatrix.inverse();
		gettimeofday(&endTime, NULL);
	}
	else
	{
		mzed_t *inverseMatrix = mzed_init(m_GF,(int) mapParameters->m_k,(int) mapParameters->m_k);

		gettimeofday(&startTime, NULL);
		mzed_invert_newton_john(inverseMatrix, mapParameters->m_vectorMatrixGf);
		gettimeofday(&endTime, NULL);

		mzed_free (inverseMatrix);
	}

	for (u_int8_t r = 0; r < mapParameters->m_rxBuffer.size(); r++)					// Sending the packet to the upper layers
	{
		item = mapParameters->m_rxBuffer[r];

		sourcePort = item.sourcePort;
		destinationPort = item.destinationPort;
		UdpHeader udpHeader;

		item.packet->RemoveHeader(udpHeader); 				// The packet is empty, so eliminate a fake udpHeader and add a real udpHeader...
		udpHeader.SetSourcePort(sourcePort);
		udpHeader.SetDestinationPort(destinationPort);
		item.packet->AddHeader(udpHeader);


		item =  mapParameters->m_rxBuffer[r];
//		if (!m_ncCallback.IsNull())
//		{
//			m_ncCallback(copy, 4, m_node->GetId(), header.GetSource(), header.GetDestination());
//		}
		m_upUdpTarget (item.packet,header,incomingInterface);

		//Statistics
		m_stats.rxBytes += mapParameters->m_packetSize;
		m_stats.rxTimestamp.push_back(Simulator::Now ().GetSeconds ());
	}

	// Increase the ACK count
	mapParameters->m_fragmentNumber = mapParameters->m_fragmentNumber +1;
	mapParameters->m_rank = 0;
	ResetMatrices(flowId);

	SendAck (header.GetSource(), header.GetDestination(), sourcePort, destinationPort, true);
	for( int d = 0; d < mapParameters->m_k; d++)
	{
		mapParameters->m_rxBuffer.pop_back();
	}


}

void IntraFlowNetworkCodingProtocol::ReduceBuffer (u_int16_t flowId)
{
	NS_LOG_FUNCTION (Simulator::Now().GetSeconds() << this );
	IntraFlowMapIterator it;

	it = m_mapParameters.find(flowId);
	if (it != m_mapParameters.end())
	{
		it->second->m_k = it->second->m_txBuffer.size();
		Encode(flowId);
	}
}

void IntraFlowNetworkCodingProtocol::GenerateRandomVector(u_int16_t K, std::vector<u_int8_t>& randomVector)
{
	NS_LOG_FUNCTION_NOARGS();

	std::vector <u_int8_t> zeros ((K), 0);
	bool exit = false;							//Exit condition

	while (! exit) // Condition to avoid sending null vectors, it will be noticed for low k values
	{
		for (u_int16_t i=0 ; i < (K) ; i++ )
		{
			int value = rand() % (int) pow(2,m_q);
			randomVector.push_back(value);
		}
		if (randomVector == zeros)
		{
			randomVector.clear();
		}
		else
		{
			exit = true;
		}
	}
	zeros.clear();
}

enum IpL4Protocol::RxStatus IntraFlowNetworkCodingProtocol::Receive (Ptr<Packet> packet, Ipv6Header const &header, Ptr<Ipv6Interface> incomingInterface){

	NS_LOG_FUNCTION (Simulator::Now().GetSeconds() << this );
	return IpL4Protocol::RX_OK;
}

enum IpL4Protocol::RxStatus IntraFlowNetworkCodingProtocol::Receive (Ptr<Packet> packet, Ipv4Header const &header, Ptr<Ipv4Interface> incomingInterface)
{
	NS_LOG_FUNCTION (m_node->GetId() << Simulator::Now().GetSeconds() << this );

	u_int16_t flowId;
	IntraFlowNetworkCodingHeader ncHeader; 			   // Once we have the header of the arriving packet, it is necessary to insert the random vector in the matrix
	Ptr <IntraFlowNetworkCodingMapParameters> mapParameters;
	IntraFlowMapIterator it;
	packet->RemoveHeader(ncHeader);

	flowId= HashID (header.GetSource(), header.GetDestination(), ncHeader.GetSourcePort(), ncHeader.GetDestinationPort());
	it = m_mapParameters.find (flowId);

	if (it == m_mapParameters.end()) //	If there is no flow registered with this flow ID, create a new entry.
	{
		NS_LOG_INFO(Simulator::Now().GetSeconds() << " Receiving new flow. @src: " << header.GetSource() << "/" << ncHeader.GetSourcePort() <<  " @dst " << header.GetDestination() << "/" << ncHeader.GetDestinationPort() << "->" <<flowId);

		Ptr<IntraFlowNetworkCodingMapParameters> aux = CreateObject<IntraFlowNetworkCodingMapParameters> ();
		aux->m_k = ncHeader.GetK();
		aux->m_rank = 0;
		aux->m_fragmentNumber = 0;
		aux->m_forwardingNode = false;
		aux->m_packetSize = packet->GetSize();

		if(m_q > 1)
		{
			m_GF = gf2e_init(irreducible_polynomials[m_q][1]);
			aux->m_vectorMatrixGf =  mzed_init(m_GF, aux->m_k,aux->m_k);
		}
		m_mapParameters.insert(pair<u_int16_t, Ptr<IntraFlowNetworkCodingMapParameters> >(make_pair(flowId, aux)));
		ResetMatrices (flowId);

		//Statistics
		m_stats.forwarder = false;
		m_stats.recode = m_recode;
	}

	it = m_mapParameters.find(flowId);
	mapParameters = it->second;

	if ((ncHeader.GetTx() & 1) == 0) // Reception of a Data packet
	{
		u_int8_t actualRank;

		if (mapParameters->m_k != ncHeader.GetK())  //The new fragment reduce the block size
		{
			mapParameters->m_k = ncHeader.GetK();
			ResetMatrices (flowId);
		}

		if(ncHeader.GetNfrag() >= mapParameters->m_fragmentNumber)
		{
			std::vector <u_int8_t> randomVector;
			randomVector = ncHeader.GetVector(); // Get the vector in the header read in "deserialized"

			if(m_q==1) //IT++ library
			{
				itpp::bvec headerVector;
				headerVector.zeros();

				for(u_int8_t i = 0; i < randomVector.size(); i++)
				{
					int valuen= randomVector[i];
					headerVector.ins (i,valuen);
				}

				mapParameters->m_vectorMatrix.set_row(mapParameters->m_rank, headerVector); // Once we have the packet header, the random vector is inserted in the matrix
				actualRank = mapParameters->m_vectorMatrix.row_rank();

				headerVector.clear();
			}
			else //M4RIE library
			{
				InsertRowGFMatrix(mapParameters->m_vectorMatrixGf, &randomVector, mapParameters->m_rank);

				mzed_t  *copy_ = mzed_init(mapParameters->m_vectorMatrixGf->finite_field, mapParameters->m_k, mapParameters->m_k);
				copy_ = mzed_copy(copy_,mapParameters->m_vectorMatrixGf);
				actualRank = mzed_echelonize(copy_,1);

				mzed_free(copy_);
			}

//			if (!m_ncCallback.IsNull())
//			{
//				m_ncCallback(packet->Copy(), 2, m_node->GetId(), header.GetSource(), header.GetDestination());
//			}

			//Statistics
			m_stats.source = false;
			m_stats.rxPackets ++;
			m_stats.packetLenght = packet->GetSize();

			NS_LOG_INFO("Receive, matrix rank: "<< actualRank);

			if (actualRank > mapParameters->m_rank)  // Check the linear independence of the vector and the matrix by using the rank
			{
				mapParameters->m_rank++; // If it is linear independent the row is incremented to fill the next one
				mapParameters->m_rxBuffer.push_back (IntraFlowNetworkCodingBufferItem(packet, header.GetSource(),header.GetDestination(), ncHeader.GetSourcePort(), ncHeader.GetDestinationPort() ));
				if (actualRank == mapParameters->m_k)
				{
					Decode (header, incomingInterface, flowId); // The matrix is full and the inverse is made by the function Decode
				}
			}


		}
		else //If the RX node receives a packet whose nFrag is lower than the one this entity is expecting, it will immediately send a forced ACK, aiming to warn the TX node that it has not updated correctly its fragment number
		{
//			if (!m_ncCallback.IsNull())
//			{
//				m_ncCallback(packet->Copy(), 9, m_node->GetId(), header.GetSource(), header.GetDestination());
//			}
			//Statistics
			m_stats.rxDiscacerdedPackets++;
			SendAck (header.GetSource(), header.GetDestination(), ncHeader.GetSourcePort(), ncHeader.GetDestinationPort(), true);
		}
	}

	else // Reception of an ACK
	{
		NS_LOG_FUNCTION (m_node->GetId() << "ReceiveACK " << ncHeader.GetNfrag()  << " " << mapParameters->m_fragmentNumber );


//		if (!m_ncCallback.IsNull())
//		{
//			m_ncCallback (packet, 3, m_node->GetId(), header.GetSource(), header.GetDestination());
//		}

		flowId= HashID (header.GetDestination(), header.GetSource(), ncHeader.GetDestinationPort(), ncHeader.GetSourcePort());
//		cout << header.GetDestination() << header.GetSource() << ncHeader.GetDestinationPort() << ncHeader.GetSourcePort()<< " " << flowId << endl;

		it = m_mapParameters.find (flowId);
		mapParameters = it->second;


		if(ncHeader.GetNfrag() > mapParameters->m_fragmentNumber)
		{
			ChangeFragment (ncHeader.GetNfrag(), flowId, false);
		}
	}
	return IpL4Protocol::RX_OK;
}


void IntraFlowNetworkCodingProtocol::ParseForwardingReception (Ptr<Ipv4Route> rtentry, Ptr<const Packet> packet, const Ipv4Header &header)
{

	NS_LOG_FUNCTION ( m_node->GetId() << Simulator::Now().GetSeconds() << this);

//	cout << "Parsing " << packet->GetUid() << endl;

	u_int16_t flowId;
	IntraFlowNetworkCodingHeader ncHeader; 			   // Once we have the header of the arriving packet, it is necessary to insert the random vector in the matrix
	Ptr <IntraFlowNetworkCodingMapParameters> mapParameters;
	IntraFlowMapIterator it;

	Ptr<Packet> copy = packet->Copy();
	copy->RemoveHeader(ncHeader);

	std::vector <u_int8_t> randomVector;

	itpp::bvec headerVector;
	headerVector.zeros ();
	if(ncHeader.GetTx() == 0)	// Data packet
	{
		u_int8_t actualRank;
		// Map creation of a new flow ID
		flowId = HashID (header.GetSource(), header.GetDestination(), ncHeader.GetSourcePort(), ncHeader.GetDestinationPort());
		it = m_mapParameters.find (flowId);

		if (it == m_mapParameters.end ())
		{
			Ptr<IntraFlowNetworkCodingMapParameters> aux = CreateObject<IntraFlowNetworkCodingMapParameters> ();
			aux->m_k = ncHeader.GetK(); 		//The intermediate node will known the fragment size from the value extracted from the NC header
			aux->m_rank = 0;
			aux->m_fragmentNumber = ncHeader.GetNfrag();
			aux->m_forwardingNode = true;
			aux->m_packetSize = copy->GetSize();

			if(m_q > 1)
			{
				m_GF = gf2e_init(irreducible_polynomials[m_q][1]);
				aux->m_vectorMatrixGf =  mzed_init(m_GF, aux->m_k,aux->m_k);

			}

			m_mapParameters.insert (pair<u_int16_t, Ptr<IntraFlowNetworkCodingMapParameters> >(make_pair(flowId, aux)));
			if (m_recode) //Initialize the matrices
			{
				ResetMatrices (flowId);
			}

			//Statistics
			m_stats.forwarder = true;
			m_stats.source = false;
			m_stats.recode = m_recode;
		}
		it = m_mapParameters.find (flowId);
		mapParameters = it->second;
		if(m_recode)  // RLNC scheme
		{
			if (ncHeader.GetNfrag() > mapParameters->m_fragmentNumber)
			{
				ChangeFragment(ncHeader.GetNfrag(),flowId,true);
			}

			if(ncHeader.GetNfrag() == mapParameters->m_fragmentNumber)  //Is my block number the same
			{
				if (mapParameters->m_k != ncHeader.GetK())  //The new fragment reduce the block size
				{
					mapParameters->m_k = ncHeader.GetK();
					ResetMatrices (flowId);
				}
				if (mapParameters->m_rank < m_k)  //The matrix is not full rank
				{
					randomVector = ncHeader.GetVector(); // Get the vector in the header read in "deserialized"
					if(m_q == 1)		//IT++ library
					{
						//Compose the random vector
						for(u_int8_t i=0; i < ncHeader.GetK(); i++)
						{
							int valuen= randomVector[i];
							headerVector.ins(i,valuen);
						}

//						cout << "Random Vector ("<< ncHeader.GetK() << "): ";
//						for (uint8_t i = 0; i < ncHeader.GetK(); i++)
//						{
//							cout << (uint32_t )randomVector[i] << " ";
//						}
//						cout << endl;

						mapParameters->m_vectorMatrix.set_row (mapParameters->m_rank, headerVector); // Once we have the packet header, the random vector is inserted in the matrix
						actualRank = mapParameters->m_vectorMatrix.row_rank();
					}
					else //M4RIE library
					{
						InsertRowGFMatrix(mapParameters->m_vectorMatrixGf,&randomVector,mapParameters->m_rank);
						mzed_t  *copy_ = mzed_init(mapParameters->m_vectorMatrixGf->finite_field, mapParameters->m_k, mapParameters->m_k);
						copy_ = mzed_copy(copy_,mapParameters->m_vectorMatrixGf);
						actualRank = mzed_echelonize(copy_,1);
						mzed_free (copy_);
					}
//					cout << "Actual Rank  " << (int) actualRank << endl;
					if (actualRank > mapParameters->m_rank)  // Check the linear independence of the vector and the matrix using the rank
					{
						mapParameters->m_receivedPackets++;
						mapParameters->m_rank++; // If it is linear independent the row is incremented to fill the next one
						mapParameters->m_txBuffer.push_back (IntraFlowNetworkCodingBufferItem(copy, header.GetSource(),header.GetDestination(), ncHeader.GetSourcePort(), ncHeader.GetDestinationPort() ));
//						if (!m_ncCallback.IsNull())
//						{
//							m_ncCallback(packet->Copy(), 6, m_node->GetId(),mapParameters->m_txBuffer[0].source,mapParameters->m_txBuffer[0].destination);
//						}
						Recode(flowId,true);
					}
				}
				else   //Matrix is full rank, The forwarding node has all the information
				{
					Recode(flowId,true);
				}

				//Satistics
				m_stats.rxPackets ++;
				m_stats.packetLenght = copy->GetSize();
			}
			else
			{
				m_stats.rxDiscacerdedPackets ++;
				SendAck (header.GetSource(), header.GetDestination(), ncHeader.GetSourcePort(), ncHeader.GetDestinationPort(), true);
				SelectiveFlushWifiBuffer(flowId);  //Remove all the packets in the buffer
			}
		}
		else // RLSC scheme --> Just forward
		{
			// Refresh the fragment number and deliver the packets to the lower layers
			if (ncHeader.GetNfrag() > mapParameters->m_fragmentNumber)
			{
				ChangeFragment(ncHeader.GetNfrag(),flowId,true);
			}

			if(ncHeader.GetNfrag() == mapParameters->m_fragmentNumber)  //Is my block number the same
			{
				Ptr<Ipv4L3Protocol> ipv4 = m_node->GetObject <Ipv4L3Protocol > ();
				if (ipv4)
				{
					Ptr<Packet> forwardedPacket = packet->Copy();
					IpL4Protocol::DownTargetCallback downTarget = GetDownTarget();
					downTarget (forwardedPacket, header.GetSource(),header.GetDestination(), IntraFlowNetworkCodingProtocol::PROT_NUMBER, 0);
					//Satistics
					m_stats.rxPackets ++;
					m_stats.packetLenght = copy->GetSize();
				}
			}
			else
			{
				//Statistics
				m_stats.rxDiscacerdedPackets ++;

				SendAck (header.GetSource(), header.GetDestination(), ncHeader.GetSourcePort(), ncHeader.GetDestinationPort(), true);
				SelectiveFlushWifiBuffer (flowId);		// Flushing the correct flow of data of the buffer
			}
		}
	}
	else // ACK
	{
//		cout << "Parsing ACK"  <<header.GetDestination() << " " << header.GetSource()<< " " << ncHeader.GetDestinationPort() << " " << ncHeader.GetSourcePort() << endl;
		flowId = HashID (header.GetDestination(), header.GetSource(), ncHeader.GetDestinationPort(), ncHeader.GetSourcePort());
		it = m_mapParameters.find(flowId);
//		cout << "Parsing ACK" << endl;
		if(ncHeader.GetNfrag() > it->second->m_fragmentNumber)
		{
			ChangeFragment (ncHeader.GetNfrag(), flowId, true);
		}
//		cout << "Forwarding Packet" << endl;
		//Forward the packet
		Ptr<Ipv4L3Protocol> ipv4 = m_node->GetObject <Ipv4L3Protocol > ();
		if (ipv4)
		{
//			cout << "Sending ACK "<< packet->GetUid() << endl;
			Ptr<Packet> forwardedPacket = packet->Copy();
//			ipv4->SendRealOutHook (rtentry, forwardedPacket, header);

			IpL4Protocol::DownTargetCallback downTarget = GetDownTarget();
			downTarget (forwardedPacket, header.GetSource(),header.GetDestination(), IntraFlowNetworkCodingProtocol::PROT_NUMBER, 0);

		}
	}
}

void IntraFlowNetworkCodingProtocol::ChangeFragment (u_int16_t nFrag, u_int16_t flowId, bool forwardingNode)
{
	NS_LOG_FUNCTION (m_node->GetId() << "--- Change fragment::IN - " << flowId << " " << nFrag << this);

	IntraFlowMapIterator it;
	Ptr <IntraFlowNetworkCodingMapParameters> mapParameters;
	it=m_mapParameters.find(flowId);
	mapParameters=it->second;

	if (forwardingNode)
	{
		mapParameters->m_txBuffer.clear();
		mapParameters->m_k = m_k;
		mapParameters->m_fragmentNumber = nFrag;			// Taking the number of fragment
		mapParameters->m_rank= 0;                           // Zeroing the rank of the matrix

		mapParameters->m_receivedPackets -= mapParameters->m_k;   //Reduce the packets already decoded

		//Start over the matrices (only with a RLNC scheme)
		if (m_recode)
		{
			ResetMatrices (flowId);
		}
//		SelectiveFlushWifiBuffer(flowId);

	}
 	else //Source nodes operation
	{
		if (nFrag > mapParameters->m_fragmentNumber)			// Erasure of the buffer
		{
			mapParameters->m_receivedPackets -= mapParameters->m_k;   //Reduce the packets already decoded
			mapParameters->m_rank= 0;

//			cout << "Reduce Buffer "<< mapParameters->m_receivedPackets << endl;
			SelectiveFlushWifiBuffer(flowId);

			mapParameters->m_fragmentNumber = nFrag; 	// It is necessary to refresh the number of fragment
			if ( mapParameters->m_receivedPackets >= m_k)
			{
				mapParameters->m_k = m_k;
				SelectiveFlushWifiBuffer(flowId);
				Encode(flowId);
			}
			else if (mapParameters->m_receivedPackets < m_k)
			{
				if(mapParameters->m_receivedPackets  != 0)
				{
					m_reduceBufferEvent = Simulator::Schedule (m_bufferTimeout, &IntraFlowNetworkCodingProtocol::ReduceBuffer, this, flowId); // It is necessary to include a timer to send the rest of the packets in case there are less than mapParameters->m_k. There is a specific function for that purpose.
				}
				else
				{
					NS_LOG_UNCOND("THERE ARE NO MORE PACKETS");
				}
			}
		}
	}
}

void IntraFlowNetworkCodingProtocol::SendAck (Ipv4Address source, Ipv4Address destination, u_int16_t sourcePort, u_int16_t destinationPort, bool normalAck)
{
	NS_LOG_FUNCTION (m_node->GetId() << Simulator::Now().GetSeconds() << this );

	Ptr<Packet> packet; // Packet creation
	packet = Create <Packet> (0);
	IntraFlowNetworkCodingHeader ncHeader;
	IntraFlowNetworkCodingBufferItem item;
	u_int16_t flowId;

	IntraFlowMapIterator it;
	Ptr <IntraFlowNetworkCodingMapParameters> mapParameters;

	flowId = HashID (source, destination, sourcePort, destinationPort);

	it=m_mapParameters.find(flowId);
	mapParameters=it->second;

	ncHeader.SetTx(1);
	ncHeader.SetK (0);  // K=0 because there is no vector in the MORE header so it only reads the useful fields
	ncHeader.SetQ (m_q);
	ncHeader.SetNfrag (mapParameters->m_fragmentNumber);
	ncHeader.SetSourcePort (destinationPort);
	ncHeader.SetDestinationPort (sourcePort);

	packet->AddHeader (ncHeader); // Add the header to the packet

	uint32_t queueSize =  (uint32_t) DynamicCast<RegularWifiMac>(m_node->GetDevice(0)->GetObject<WifiNetDevice>()->GetMac())->GetDcaTxopPub()->GetQueue()->GetSize();
//	cout << "QUEUE Size" <<  queueSize << endl;
 	if (queueSize > 0) //Check if there is more than one ACK packet in the buffer, avoiding send multiple ACK packets
		return;

//	if (!m_ncCallback.IsNull())
//	{
//		m_ncCallback(packet, 5, m_node->GetId(), destination, source);
//	}

//	cout << m_node->GetId()   << " Send ACK" << destination <<  " " << source << " " << packet->GetUid()<< endl;
	IpL4Protocol::DownTargetCallback downTarget = GetDownTarget();
	downTarget (packet, destination, source, IntraFlowNetworkCodingProtocol::PROT_NUMBER, 0); // Change the source and established the destination
}

bool IntraFlowNetworkCodingProtocol::ReceivePromiscuous (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol, const Address &from, const Address &to, NetDevice::PacketType packetType)
{
	NS_LOG_FUNCTION( m_node->GetId() << this << packet->GetUid());

	//Check whether the packet is headed to us
	//First, we do need to get the destination IP address from the IP header
	Ptr<Packet> copy = packet->Copy();
	Ipv4Header ipHeader;

	switch (protocol)
	{
	case 0x800:  //IP packet
		copy->RemoveHeader (ipHeader);
		//Second -> Check whether this object belongs to the destination address
		//Third, process only those packets that carry IntraFlowNetworkCodingProtocol
		switch (ipHeader.GetProtocol())
		{
		case IntraFlowNetworkCodingProtocol::PROT_NUMBER:
		{
			IntraFlowNetworkCodingHeader ncHeader;
			copy->PeekHeader(ncHeader);

			if (AmIDestination (ipHeader.GetDestination()) && packetType!=NetDevice::PACKET_HOST)   //OverHearing by the destination
			{
				if (m_overhearing)
				{
					Receive(copy, ipHeader, m_node->GetObject<Ipv4L3Protocol>()->GetInterface (m_node->GetObject<Ipv4L3Protocol>()->GetInterfaceForDevice(device)));

				}
			}
			if (!AmIDestination(ipHeader.GetDestination()) && !AmISource(ipHeader.GetSource()))
			{
				if ((ncHeader.GetTx() & 1) == 0) //Data packet
				{
					//Parse the number of hops (TTL field in IP header) in order to avoid flooding
					Ptr<Ipv4L3Protocol> ipv4 = m_node->GetObject <Ipv4L3Protocol > ();
					if (ipv4)
					{
						Socket::SocketErrno errno_;
						Ptr<NetDevice> oif(0); // unused for now
						Ptr<Ipv4Route> route;
						route  = ipv4->GetRoutingProtocol()->RouteOutput(copy, ipHeader, oif, errno_);
						if (route)
						{
							if (m_overhearing)
							{
								ParseForwardingReception(route, copy, ipHeader);

							}
						}
						else
						{
							if (m_overhearing)
							{
								ParseForwardingReception(0, copy, ipHeader);

							}
						}
					}
				}
				else //Relay nodes must parse the ACKs as well
				{
					IntraFlowMapIterator it;
					u_int16_t flowId;
					flowId = HashID(ipHeader.GetDestination(), ipHeader.GetSource(), ncHeader.GetDestinationPort(), ncHeader.GetSourcePort()); 					//Ack inversion of Destionation and Source
					it = m_mapParameters.find (flowId);
					if (it != m_mapParameters.end())
					{
						if(ncHeader.GetNfrag() > it->second->m_fragmentNumber)
						{
							ChangeFragment(ncHeader.GetNfrag(), flowId, true);
						}
//						if (m_overhearing)
						{
							ParseForwardingReception(0, copy, ipHeader);

						}
					}
				}
			}
			break;
		}
		default:
			break;
		}
		break;
	default:
		break;
	}
	return true;
}

bool IntraFlowNetworkCodingProtocol::AmIDestination(const Ipv4Address &destination)
{
	NS_LOG_FUNCTION(this);
	return (destination == m_node->GetObject<Ipv4 > ()->GetAddress(1, 0).GetLocal());
}

bool IntraFlowNetworkCodingProtocol::AmISource(const Ipv4Address &source)
{
	NS_LOG_FUNCTION(this);
	return (source == m_node->GetObject<Ipv4 > ()->GetAddress(1, 0).GetLocal());
}

void IntraFlowNetworkCodingProtocol::FlushWifiBuffer ()
{
	m_flushCallback ();
}

void IntraFlowNetworkCodingProtocol::SelectiveFlushWifiBuffer (u_int16_t flowId)
{
	m_selectiveFlushCallback (flowId);
}

void IntraFlowNetworkCodingProtocol::WifiBufferEvent (Ptr<const Packet> packet)
{
	WifiMacHeader macHeader;
	LlcSnapHeader llcHeader;
	Ipv4Header ipHeader;
	IntraFlowNetworkCodingHeader ncHeader;
	UdpHeader udpHeader;

	u_int16_t flowId;

	Ptr<Packet> copy = packet->Copy();
	copy->RemoveHeader(macHeader);

	//Identify flows in order to keep track of the WifiMacQueue size
	if (macHeader.IsData() && !macHeader.GetAddr1().IsBroadcast() && !macHeader.IsRetry())
	{
		copy->RemoveHeader (llcHeader);
		switch (llcHeader.GetType())
		{
		case 0x0806:            //ARP
			break;
		case 0x0800:            //IP packet
			copy->RemoveHeader(ipHeader);
			switch (ipHeader.GetProtocol())
			{
			case 6:             //TCP
			case 17:            //UDP
			case 99:    //Network Coding --> Force the shortest frames to be correct
				break;
			case 100:
			{
				Ptr<Packet> copy2 = copy->Copy();
				copy->RemoveHeader(ncHeader);
				if ((ncHeader.GetTx() & 1) == 0)        //Data packets
				{
					//Look up if the output packet is already stored into any of the buffers
					flowId = HashID (ipHeader.GetSource(), ipHeader.GetDestination(), ncHeader.GetSourcePort(), ncHeader.GetDestinationPort());
					IntraFlowMapIterator iter = m_mapParameters.find(flowId);

					if (iter != m_mapParameters.end())
					{
						//Satistics
						m_stats.txPackets ++;
						m_stats.txBytes += copy2->GetSize();
						m_stats.txTimestamp.push_back(Simulator::Now ().GetSeconds ());

						if(iter->second->m_forwardingNode == false)
						{
							Encode(flowId);
						}
						else if (m_recode)
						{
							//SelectiveFlushWifiBuffer(flowId)
							//Recode(flowId, false);    //Each time a forwarder has the oportunity of transmit -> Generate a new packet
						}
					}
				}
				else{

				}
				break;
			}
			default:
				break;
			}
			break;
			default:
				break;
		}
	}
}




void IntraFlowNetworkCodingProtocol::ResetMatrices (u_int16_t flowId)
{
	IntraFlowMapIterator iter = m_mapParameters.find (flowId);

	if (iter != m_mapParameters.end())
	{
		if(m_q==1)						// Overwriting with the new matrix
		{
			iter->second->m_vectorMatrix = GF2mat (iter->second->m_k, iter->second->m_k);
		}
		else
		{
			mzed_free(iter->second->m_vectorMatrixGf);
			iter->second->m_vectorMatrixGf = mzed_init(m_GF, iter->second->m_k,iter->second->m_k);
		}
		iter->second->m_rank = 0;
	}

}

void IntraFlowNetworkCodingProtocol::InsertRowGFMatrix(mzed_t *A, vector<u_int8_t> *vector, int rowNumber){

    std::vector<u_int8_t>::iterator it;
    int colum = 0;
    for (it= vector->begin(); it != vector->end(); it++) {
        word m = *it;
        mzed_write_elem(A,rowNumber,colum, m);
        colum++;
    }
}
