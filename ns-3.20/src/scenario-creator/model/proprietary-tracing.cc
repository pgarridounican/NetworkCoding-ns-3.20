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
 * Author: Pablo Garrido Ortiz <pgarrido@tlmat.unican.es>
 * 		   David Gómez Fernández <dgomez@tlmat.unican.es>
 *         Ramón Agüero Calvo <ramon@tlmat.unican.es>
 */

#include "proprietary-tracing.h"
#include "ns3/config.h"

//Needed to parse the packet content (BurstyErrorModel::ParsePacket)
#include "ns3/wifi-mac-header.h"
#include "ns3/llc-snap-header.h"
#include "ns3/ipv4-header.h"
#include "ns3/tcp-header.h"
#include "ns3/udp-header.h"

#include "ns3/tag.h"

#include "ns3/onoff-application.h"
#include "ns3/packet-sink.h"

#include <algorithm>
#include <numeric>
#include <vector>
#include <iterator>
#include <unistd.h>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("ProprietaryTracing");
NS_OBJECT_ENSURE_REGISTERED(ProprietaryTracing);


TracingInformation::TracingInformation()
{
	run = 0;
	totalRuns = 0;
	runOffset = 0;
	applicationStartTime = Seconds(20.0);

	packetLength = 0;
	numPackets = 0;
}

TracingInformation::~TracingInformation()
{

}

ProprietaryTracing::ProprietaryTracing ()
{
    NS_LOG_FUNCTION(this);
//    m_totalDataPackets = 0;
//    m_totalDataCorrectPackets = 0;
//    m_totalDataCorruptedPackets = 0;
//    m_txBytes = 0;
//    m_rxBytes = 0;
//    m_flowMonitorEnabler = false;
}

ProprietaryTracing::~ProprietaryTracing ()
{
    NS_LOG_FUNCTION(this);


}

void ProprietaryTracing::PrintStatistics ()
{
	NS_LOG_FUNCTION (this);

	if (m_applicationLevelShortTraceFile.is_open())
	{
		PrintApplicationStatistics();
		m_applicationLevelShortTraceFile.close();
	}

	if (m_ncLevelShortTraceFile.is_open())
	{
		PrintNCStatistics();
		m_ncLevelShortTraceFile.close();
	}
	if (m_phyWifiLevelShortTracingFile.is_open())
	{
		PrintWifiPhyStatistics();
		m_phyWifiLevelShortTracingFile.close();
	}
}

////////////// APPLICATION LEVEL TRACING //////////////

void ProprietaryTracing::EnableApplicationShortTraceFile()
{
	NS_LOG_FUNCTION_NOARGS ();
	char buf[FILENAME_MAX];

	string fileName;
	string path = string (getcwd (buf, FILENAME_MAX));

	fileName = "APP_SHORT_"  + m_traceInfo.fileName + ".tr";


//	std::replace (fileName.begin(), fileName.end(), '-', '_');
	path += "/traces/" + fileName;

	//	Try to open an existing file; if error, open a new one
	m_applicationLevelShortTraceFile.open (path.c_str (), fstream::in | fstream::out | fstream::ate);

	if (m_applicationLevelShortTraceFile.fail ())
	{
		m_applicationLevelShortTraceFile.close ();
		char headerLine [FILENAME_MAX];
		m_applicationLevelShortTraceFile.open (path.c_str (), fstream::out | fstream::ate);

		sprintf (headerLine, "%8s %8s %10s %10s %10s %10s %14s %14s",
				"#", "Node", "SRC/SINK", "Pkt_len", "#TX", "#RX", "Thput(Mbps)", "Time (sec)");

		m_applicationLevelShortTraceFile << headerLine << endl;
	}

}

void ProprietaryTracing::PrintApplicationStatistics()
{
	char line [FILENAME_MAX];
	double thput;
	double elapsedTime;

	for (u_int8_t i = 0; i < NetworkMonitor::Instance().GetSourceApps().GetN(); i ++)
	{
		//Throughput (Mbps)
		NetworkMonitor::Instance().GetSourceApps().Get(i)->GetStats().txTimestamp.size() ? thput =
				(NetworkMonitor::Instance().GetSourceApps().Get(i)->GetStats().txBytes*8) /
				(NetworkMonitor::Instance().GetSourceApps().Get(i)->GetStats().txTimestamp.back() -
						NetworkMonitor::Instance().GetSourceApps().Get(i)->GetStats().txTimestamp.front()) / 1e6 : thput = 0.0;

		//Elapsed time (seconds)
		NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxTimestamp.size() ? elapsedTime =
				NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxTimestamp.back() -
				NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxTimestamp.front() : elapsedTime = 0.0;


		sprintf (line, "%8d %8d %10d %10d %10d %10d %14.4f %14.4f",
						m_traceInfo.run,
						NetworkMonitor::Instance().GetSourceApps().Get(i)->GetNode()->GetId(),
						1,
						m_traceInfo.packetLength,
						NetworkMonitor::Instance().GetSourceApps().Get(i)->GetStats().txPackets,
						NetworkMonitor::Instance().GetSourceApps().Get(i)->GetStats().rxPackets,
						thput,
						elapsedTime
				);
		m_applicationLevelShortTraceFile << line << endl;
	}

	for (u_int8_t i = 0; i < NetworkMonitor::Instance().GetSinkApps().GetN(); i ++)
	{
		//Throughput (Mbps)
		NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxTimestamp.size() ? thput =
				(NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxBytes*8) /
				(NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxTimestamp.back() -
						NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxTimestamp.front()) / 1e6 : thput = 0.0;


		//Elapsed time (seconds)
		NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxTimestamp.size() ? elapsedTime =
						NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxTimestamp.back() -
								NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxTimestamp.front() : elapsedTime = 0.0;

		sprintf (line, "%8d %8d %10d %10d %10d %10d %14.4f %14.4f",
						m_traceInfo.run,
						NetworkMonitor::Instance().GetSinkApps().Get(i)->GetNode()->GetId(),
						0,
						m_traceInfo.packetLength,
						NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().txPackets,
						NetworkMonitor::Instance().GetSinkApps().Get(i)->GetStats().rxPackets,
						thput,
						elapsedTime
				);
		m_applicationLevelShortTraceFile << line << endl;
	}
}

////////////// END APPLICATION LEVEL TRACING //////////////

////////////// NC LEVEL TRACING //////////////

void ProprietaryTracing::EnableNCShortTraceFile()
{
	NS_LOG_FUNCTION_NOARGS ();
	char buf[FILENAME_MAX];

	string fileName;
	string path = string (getcwd (buf, FILENAME_MAX));

	fileName = "NC_SHORT_"  + m_traceInfo.fileName + ".tr";


//	std::replace (fileName.begin(), fileName.end(), '-', '_');
	path += "/traces/" + fileName;

	//	Try to open an existing file; if error, open a new one
	m_ncLevelShortTraceFile.open (path.c_str (), fstream::in | fstream::out | fstream::ate);

	if (m_ncLevelShortTraceFile.fail ())
	{
		m_ncLevelShortTraceFile.close ();
		char headerLine [FILENAME_MAX];
		m_ncLevelShortTraceFile.open (path.c_str (), fstream::out | fstream::ate);

		sprintf (headerLine, "%8s %8s %8s %8s %8s %10s %10s %10s %10s %10s %14s %14s",
				"#", "Node", "SRC/SINK", "K", "q", "Recode" ,"Pkt_len", "#TX", "#RX" , "#DiscRX", "Thput(Mbps)", "Time (sec)");

		m_ncLevelShortTraceFile << headerLine << endl;
	}
}

void ProprietaryTracing::PrintNCStatistics()
{
	char line [FILENAME_MAX];
	double thput;
	double elapsedTime;
	uint32_t typeNode;

	for (u_int8_t i = 0; i < NetworkMonitor::Instance().GetNetworkCodingVectorSize(); i ++)
	{
		if (NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().source) //Source
		{
			typeNode = 0;
		}
		else if (NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().forwarder) //Forwarder
		{
			typeNode = 2;

		}
		else  //Destination
		{
			typeNode = 1;
		}
		//Throughput (Mbps)
		NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().rxTimestamp.size() ? thput =
				(NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().rxBytes*8) /
				(NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().rxTimestamp.back() -
						NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().rxTimestamp.front()) / 1e6 : thput = 0.0;

		//Elapsed time (seconds)
		NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().rxTimestamp.size() ? elapsedTime =
				NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().rxTimestamp.back() -
				NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().rxTimestamp.front() : elapsedTime = 0.0;

		sprintf (line, "%8d %8d %8d %8d %8d %10d %10d %10d %10d %10d %14.4f %14.4f",
						m_traceInfo.run,
						NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetNode()->GetId(),
						typeNode,
						NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().k,
						NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().q,
						NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().recode,
						NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().packetLenght,
//						NetworkMonitor::Instance().GetSourceApps().Get(i)->GetStats().txBytes,
						NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().txPackets,
						NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().rxPackets,
//						NetworkMonitor::Instance().GetSourceApps().Get(i)->GetStats().rxBytes,
						NetworkMonitor::Instance().GetNetworkCodingElement(i)->GetStats().rxDiscacerdedPackets,
						thput,
						elapsedTime
				);
		m_ncLevelShortTraceFile << line << endl;
	}
}

////////////// END APPLICATION LEVEL TRACING //////////////


void ProprietaryTracing::EnableWifiPhyLevelShortTracing ()
{


	NS_LOG_FUNCTION_NOARGS ();
	char buf[FILENAME_MAX];

	string fileName;
	string path = string (getcwd (buf, FILENAME_MAX));


	fileName = "PHY_SHORT_"  + m_traceInfo.fileName + ".tr";

	path += "/traces/" + fileName;

	//	Try to open an existing file; if error, open a new one
	m_phyWifiLevelShortTracingFile.open (path.c_str (), fstream::in | fstream::out | fstream::ate);

	if (m_phyWifiLevelShortTracingFile.fail ())
	{
		cout << "Traces" << endl;
		m_phyWifiLevelShortTracingFile.close ();
		char headerLine [FILENAME_MAX];
		m_phyWifiLevelShortTracingFile.open (path.c_str (), fstream::out | fstream::ate);

		sprintf (headerLine, "%10s %10s %10s %10s %10s %10s %10s %10s",
				"#", "Node", "#Tx", "#Rx", "RxT", "#RxDrops", "#RxError", "RxCol");

		m_phyWifiLevelShortTracingFile << headerLine << endl;
	}
}

void ProprietaryTracing::PrintWifiPhyStatistics()
{
	NS_LOG_FUNCTION_NOARGS ();

	char line [FILENAME_MAX];

	for (uint32_t i = 0; i < NodeList::GetNNodes(); i++)
	{
		for (uint32_t j=0; j < NodeList::GetNode(i)->GetNDevices(); j++)
		{
			Ptr<WifiNetDevice> wifiDevice = NodeList::GetNode(i)->GetDevice(j)->GetObject<WifiNetDevice>();
			if(wifiDevice)
			{
				Ptr<WifiPhy> wifi = wifiDevice->GetPhy();
				sprintf (line, "%10d %10d %10d %10d %10d %10d %10d %10d",
						m_traceInfo.run,
						i,
						wifi->GetStats().txPackets,
						wifi->GetStats().rxPackets,
						wifi->GetStats().rxEndPackets,
						wifi->GetStats().rxDropPackets,
						wifi->GetStats().rxErrorPackets,
						wifi->GetStats().rxCollisionPackets
						);

				m_phyWifiLevelShortTracingFile << line << endl;
			}
		}
	}
}


void WifiTxBeginEvent (Ptr<const Packet> packet)
{
	NS_LOG_FUNCTION_NOARGS ();

}

void WifiTxDropEvent (Ptr<const Packet> packet)
{

}

void WifiTxEndEvent (Ptr<const Packet> packet)
{

}

void WifiRxBeginEvent (Ptr<const Packet> packet)
{

}

void WifiRxDropEvent (Ptr<const Packet> packet)
{

}

void WifiRxEndEvent (Ptr<const Packet> packet)
{

}
