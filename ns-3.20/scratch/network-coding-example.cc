/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 Universidad de Cantabria
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
 * Author: David Gómez Fernández <dgomez@tlmat.unican.es>
 *		   Ramón Agüero Calvo <ramon@tlmat.unican.es>
 */

#include <sys/types.h>
#include <assert.h>

#include "ns3/node-list.h"
#include "scratch-logging.h"				//Set the LOGGING options

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-loss-model.h"

#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/inet-socket-address.h"

#include "ns3/network-coding-helper.h"
#include "ns3/network-coding-l4-protocol.h"
#include "ns3/intra-flow-network-coding-protocol.h"
#include "ns3/error-model.h"



#include "ns3/network-monitor.h"
#include "ns3/proprietary-tracing.h"

using namespace ns3;
using namespace std;


/**
 * Simple script to test the scenario-creator handler. User only need the following stuff:
 * 		- Raw file name where we have stored the scenario configuration values. NOTE: it MUST have the ".config" extension, but MUST NOT be included in the variable declaration.
 *
 * To run the script, just prompt a command similar to this one: ./waf --run "scratch/test-scenario --Configuration=network-coding-scenario"
 *
 * ENJOY!!
 */

int main(int argc, char *argv[]) {


	clock_t begin, end;

    u_int32_t m_runs = 1;
    u_int32_t m_runCounter;
    u_int32_t m_runOffset = 0;

    //Wifi Parameters
    uint8_t m_retransmissions = 1;

    //Link Errors
    double m_ferS_R1 = 0.3;
    double m_ferS_R2 = 0.3;
    double m_ferR1_D = 0.3;
    double m_ferR2_D = 0.3;

    //NC parameters
    bool m_nc = true;
    uint8_t m_k = 64;
    uint8_t m_q = 1;
    bool m_recoding = true;
    Time m_codingBufferTimeout = Seconds(100);
    bool m_overhearing = false;
    bool m_systematic = false;
    uint16_t m_startRecoding = 64;

    //Transport Protocol
    string m_transportProtocol = "ns3::TcpSocketFactory";
    if (m_nc == true)
    	m_transportProtocol = "ns3::UdpSocketFactory";

    //App paramters
    uint32_t m_packetLenght =1500 - 20 - 8 - 9 - ceil(m_k*m_q/8); //Maximum value if we work with 802.11b
//    uint32_t m_packetLenght = 1460; //Maximum value if we work with 802.11b
    uint32_t m_numberPackets = 6400;
    string dataRate = "10Mbps";
    DataRateValue m_dataRate = DataRateValue(dataRate);


    //Activate the logging  (from the library scratch-logging.h, just modify there those LOGGERS as wanted)
    EnableLogging();

    for (m_runCounter = m_runOffset; m_runCounter < m_runs; m_runCounter++) {

    	SeedManager::SetRun(m_runCounter);
    	SeedManager::SetSeed(m_runCounter+1);
    	begin = clock();

    	//Set Attributes
    	//Propagation Attributes
    	Config::SetDefault("ns3::RangePropagationLossModel::MaxRange", DoubleValue(20.0));

    	//ARP Attributes (by default, it will perpetually store the entries)
    	Config::SetDefault ("ns3::ArpCache::AliveTimeout", TimeValue(Seconds(10000)));
    	Config::SetDefault ("ns3::ArpCache::DeadTimeout", TimeValue(Seconds(10000)));

    	//Wifi Attributes
    	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue("DsssRate2Mbps"));
    	Config::SetDefault ("ns3::YansWifiPhy::TxPowerStart", DoubleValue(0.0));
    	Config::SetDefault ("ns3::YansWifiPhy::TxPowerEnd", DoubleValue(0.0));
    	Config::SetDefault ("ns3::YansWifiPhy::CcaMode1Threshold", DoubleValue(-150.0)); //CS power level

    	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));    		//Disable RTS/CTS transmission
    	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200")); 		//Disable fragmentation
    	Config::SetDefault ("ns3::ConstantRateWifiManager::DataMode", StringValue("DsssRate11Mbps"));    		//
    	Config::SetDefault ("ns3::ConstantRateWifiManager::ControlMode", StringValue("DsssRate11Mbps")); 		//
    	Config::SetDefault ("ns3::WifiMacQueue::MaxPacketNumber", UintegerValue(100000000)); 					//Maximum number of packets supported by the Wifi MAC buffer
    	Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue(Seconds(10000))); 							//Not drop packets from buffer

    	Config::SetDefault ("ns3::WifiRemoteStationManager::MaxSlrc", UintegerValue(m_retransmissions));		//Maximum number of transmission attempts
    	Config::SetDefault ("ns3::WifiNetDevice::Mtu", UintegerValue(1500));   									//

    	//NC Attributes
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::Q", UintegerValue(m_q));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::K",UintegerValue(m_k));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::Recoding", BooleanValue (m_recoding));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::BufferTimeout", TimeValue(m_codingBufferTimeout));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::OverHearing", BooleanValue(m_overhearing));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::Systematic", BooleanValue(m_systematic));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::StartRecoding", UintegerValue(m_startRecoding));

    	//Transport-level Attributes
    	if (m_transportProtocol == "ns3::TcpSocketFactory") // TCP Attributes
    	{
    		Config::SetDefault ("ns3::TcpSocket::TcpNoDelay", BooleanValue(true));
    		Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1460));
    		Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(90000000));
    		Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(90000000));
    	}
    	else
    	{
    		Config::SetDefault("ns3::UdpSocket::RcvBufSize", UintegerValue(90000000));
//    		Config::SetDefault("ns3::UdpSocket::SndBufSize", UintegerValue(90000000));
    	}

    	//PacketSocket buffer
    	Config::SetDefault ("ns3::PacketSocket::RcvBufSize", UintegerValue(90000000));

    	//Application layer
    	Config::SetDefault("ns3::OnOffApplication::DataRate", DataRateValue(m_dataRate));


    	//Deploy scenario

    	NodeContainer nodeContainer;
    	nodeContainer.Create(4);

    	Ptr<Node> sourceNode = nodeContainer.Get(0);
    	Ptr<Node> destinationNode = nodeContainer.Get(3);
    	Ptr<Node> relay1 = nodeContainer.Get(1);
    	Ptr<Node> relay2 = nodeContainer.Get(2);

    	//Position and Mobility (In fact, it does not matter)
        MobilityHelper mobilityHelper;
        Ptr <ListPositionAllocator> listPositionAllocator = CreateObject<ListPositionAllocator> ();
    	listPositionAllocator->Add(Vector(0.0,0.0,0.0));  //Source
    	listPositionAllocator->Add(Vector(0.0,8.0,0.0));  //Relay1
    	listPositionAllocator->Add(Vector(0.0,17.0,0.0)); //Relay2
    	listPositionAllocator->Add(Vector(0.0,25.0,0.0)); //Destination

        mobilityHelper.SetPositionAllocator(listPositionAllocator);
        mobilityHelper.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobilityHelper.Install (nodeContainer);

        //Wifi Interface

        WifiHelper wifiHelper;
        wifiHelper.Default ();
        wifiHelper.SetStandard(WIFI_PHY_STANDARD_80211b);
        wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager");

        NqosWifiMacHelper nQosWifiMacHelper = nQosWifiMacHelper.Default(); //By default, configure the nodes in AdHoc mode;
        nQosWifiMacHelper.SetType("ns3::AdhocWifiMac");

        YansWifiChannelHelper channelHelper;
        channelHelper.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
        channelHelper.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange",DoubleValue(20));

        // Error Model
    	Ptr<MatrixErrorModel> error = CreateObject<MatrixErrorModel> ();
    	error->SetDefaultFer (0.0);
    	error->SetFer (0,1,m_ferS_R1);
    	error->SetFer (0,2,m_ferS_R2);
    	error->SetFer (1,3,m_ferR1_D);
    	error->SetFer (2,3,m_ferR2_D);

    	error->SetFer (1,2,1.0);
    	error->SetFer (2,1,1.0);

    	error->SetFer (1,0,m_ferS_R1);
    	error->SetFer (2,0,m_ferS_R2);
    	error->SetFer (3,1,m_ferR1_D);
    	error->SetFer (3,2,m_ferR2_D);

        YansWifiPhyHelper phyHelper;
        phyHelper = YansWifiPhyHelper::Default ();
        phyHelper.SetErrorModel(error);
        phyHelper.SetChannel(channelHelper.Create());

        NetDeviceContainer devices;
        devices = wifiHelper.Install (phyHelper, nQosWifiMacHelper, nodeContainer);

        //Internet Stack Protocol (IP)
    	InternetStackHelper internet;
    	internet.Install (nodeContainer);

    	Ipv4AddressHelper ipv4;
    	ipv4.SetBase("10.1.1.0","255.255.255.0");
    	Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

        // Routing Tables
    	Ipv4StaticRoutingHelper staticRouting;
        Ipv4ListRoutingHelper list;
        list.Add(staticRouting, 0);

    	Ipv4Address srcAddress = nodeContainer.Get(0)->GetObject<Ipv4> ()->GetAddress(1, 0).GetLocal();
    	Ipv4Address relay1Address = nodeContainer.Get(1)->GetObject<Ipv4> ()->GetAddress(1, 0).GetLocal();
    	Ipv4Address relay2Address = nodeContainer.Get(2)->GetObject<Ipv4> ()->GetAddress(1, 0).GetLocal();
    	Ipv4Address dstAddress = nodeContainer.Get(3)->GetObject<Ipv4> ()->GetAddress(1, 0).GetLocal();

    	// Source
    	Ptr<Ipv4StaticRouting> routingEntry = staticRouting.GetStaticRouting(nodeContainer.Get(0)->GetObject<Ipv4 > ());
    	routingEntry->AddHostRouteTo(dstAddress, relay1Address, 1, 0);
//    	routingEntry->AddHostRouteTo(dstAddress, relay2Address, 1, 0);
    	routingEntry->AddHostRouteTo(relay1Address, relay1Address, 1, 0);
    	routingEntry->AddHostRouteTo(relay2Address, relay2Address, 1, 0);

    	//Relay1
    	routingEntry = staticRouting.GetStaticRouting(nodeContainer.Get(1)->GetObject<Ipv4 > ());
    	routingEntry->AddHostRouteTo(dstAddress, dstAddress, 1, 0);
    	routingEntry->AddHostRouteTo(srcAddress, srcAddress, 1, 0);

    	//Relay2
    	routingEntry = staticRouting.GetStaticRouting(nodeContainer.Get(2)->GetObject<Ipv4 > ());
    	routingEntry->AddHostRouteTo(dstAddress, dstAddress, 1, 0);
    	routingEntry->AddHostRouteTo(srcAddress, srcAddress, 1, 0);

    	//Destination
    	routingEntry = staticRouting.GetStaticRouting(nodeContainer.Get(3)->GetObject<Ipv4 > ());
    	routingEntry->AddHostRouteTo(srcAddress, relay1Address, 1, 0);
//    	routingEntry->AddHostRouteTo(srcAddress, relay1Address, 1, 0);
    	routingEntry->AddHostRouteTo(relay1Address, relay1Address, 1, 0);
    	routingEntry->AddHostRouteTo(relay2Address, relay2Address, 1, 0);

    	internet.SetRoutingHelper(list);

    	string fileName = "traces/route";

        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper > (fileName, std::ios::out);
        staticRouting.PrintRoutingTableAllEvery (Seconds(5), routingStream);

    	//NC
    	if(m_nc == true)
    	{
    		NetworkCodingHelper ncHelper;
    		ncHelper.Install(nodeContainer, "IntraFlowNetworkCodingProtocol");

        	//Trace Information
        	for (uint32_t i = 0; i < ncHelper.GetNetworkCodingList().size(); i++)
        	{
        	    NetworkMonitor::Instance().AddNetworkCodingLayer(ncHelper.GetNetworkCodingList()[i]);
        	}
    	}


    	//Aplication -- Full the ARP cache previously
    	double offset = 0;
        ApplicationContainer clientApps, serverApps;
        for (int i = 0; i < (int) nodeContainer.GetN(); i++) {
            UdpEchoServerHelper echoServer(9);
            serverApps = echoServer.Install(nodeContainer.Get(i));
            serverApps.Start(Seconds(1.0));
            serverApps.Stop(Seconds(20.0));


            for (int j = 0; j < (int) nodeContainer.GetN(); j++)
            {
                if (i != j)
                {
                    UdpEchoClientHelper echoClient(nodeContainer.Get(j)->GetObject<Ipv4 > ()->GetAddress(1, 0).GetLocal(), 9);
                    echoClient.SetAttribute("MaxPackets", UintegerValue(1));
                    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
                    echoClient.SetAttribute("PacketSize", UintegerValue(200));

                    clientApps = echoClient.Install(nodeContainer.Get(i));
                    clientApps.Start(Seconds(4.0 + offset));
                    clientApps.Stop(Seconds(20.0));

                    offset += 0.01;
                }
            }
        }

		//Application
	    ApplicationContainer sourceAppContainer;
	    ApplicationContainer sinkAppContainer;


	    OnOffHelper onOff (m_transportProtocol, Address(InetSocketAddress(dstAddress,50000)));
	    onOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));  //Change for ns-3.20
	    onOff.SetAttribute("OffTime",StringValue ("ns3::ConstantRandomVariable[Constant=0]")); //Change for ns-3.20
	    onOff.SetAttribute("PacketSize", UintegerValue(m_packetLenght));
	    onOff.SetAttribute("MaxBytes", UintegerValue(m_packetLenght*m_numberPackets));
	    sourceAppContainer = onOff.Install (nodeContainer.Get(0));
	    sourceAppContainer.Start(Seconds(20));
	    sourceAppContainer.Stop(Seconds(10000.0));

	    //Packet sink
	    PacketSinkHelper packetSink (m_transportProtocol, Address(InetSocketAddress (Ipv4Address::GetAny(), 50000)));
	    sinkAppContainer = packetSink.Install(nodeContainer.Get(3));
	    sinkAppContainer.Start(Seconds(20.0));
	    sinkAppContainer.Stop(Seconds(10000.0));

	    //Trace Information
	    NetworkMonitor::Instance().GetSourceApps().Add (sourceAppContainer);
	    NetworkMonitor::Instance().GetSinkApps().Add (sinkAppContainer);

	    Ptr<ProprietaryTracing> prop = CreateObject<ProprietaryTracing > ();
	    prop->GetTraceInfo().run = m_runCounter;
	    prop->GetTraceInfo().fileName = "4Nodes";
	    prop->GetTraceInfo().totalRuns = m_runs;
	    prop->GetTraceInfo().packetLength = m_packetLenght;
	    prop->GetTraceInfo().numPackets = m_numberPackets;

	    prop->EnableApplicationShortTraceFile();
	    prop->EnableNCShortTraceFile();
	    prop->EnableWifiPhyLevelShortTracing();

    	//Run the simulation
    	Simulator::Stop(Seconds (1000.0));
    	Simulator::Run ();

    	end = clock ();
    	//Print final statistics
	    prop->PrintStatistics();

    	char output[200];
		sprintf(output, "[%04.5f sec] - Run %d ", (double) (end - begin) / CLOCKS_PER_SEC,
				m_runCounter);
		printf("%s\n", output);

    	Simulator::Destroy ();
    	NetworkMonitor::Instance().Reset();
    }
    return 0;
} //end main
