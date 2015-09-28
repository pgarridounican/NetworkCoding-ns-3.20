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
 * Author: David Gómez Fernández <dgomez@tlmat.unican.es>
 *         Ramón Agüero Calvo <ramon@tlmat.unican.es>
 */

#include "configure-scenario.h"
#include "network-monitor.h"

//Network Coding implementation
#include "ns3/network-coding-helper.h"
#include "ns3/network-coding-l4-protocol.h"
#include "ns3/intra-flow-network-coding-protocol.h"
#include "ns3/error-model.h"


#include <sstream>
#include <stdio.h>
#include <sys/types.h>
#include <assert.h>

#include <map>
#include <list>

using namespace ns3;
using namespace std;

//const bool g_debug = true;  							//Temporal solution (only for debugging)

NS_LOG_COMPONENT_DEFINE("ConfigureScenario");
NS_OBJECT_ENSURE_REGISTERED(ConfigureScenario);


ConfigureScenario::ConfigureScenario()
{
    NS_LOG_FUNCTION(this);

    if (m_channelFer.size())
    {
        m_channelFer.clear();
    }

    m_configurationFile = CreateObject<ConfigurationFile> ();
    m_transportProtocol = TCP_PROTOCOL;
    m_propTracing = CreateObject<ProprietaryTracing > ();
}

ConfigureScenario::~ConfigureScenario()
{
    NS_LOG_FUNCTION(this);

    if (m_channelFer.size())
    {
        m_channelFer.clear();
    }

    //Reset the NetworkMonitor
    NetworkMonitor::Instance().Reset();

}

TypeId
ConfigureScenario::GetTypeId(void)
{
	static TypeId tid = TypeId("ns3::ConfigureScenario")
    .SetParent<Object > ()
    .AddConstructor<ConfigureScenario > ()
	;
return tid;

}

bool ConfigureScenario::ParseConfigurationFile(string confFile)
{
    NS_LOG_FUNCTION_NOARGS();
    string value;

    assert (m_configurationFile->LoadConfig(m_configurationFile->SetConfigFileName("/src/scenario-creator/config/", confFile)) != -1);

    //Get the scenario configuration from the file
    assert(m_configurationFile->GetKeyValue("SCENARIO", "RUN", value) >= 0);
    m_totalRuns = atoi(value.c_str());

    assert(m_configurationFile->GetKeyValue("SCENARIO", "RUN_OFFSET", value) >= 0);
    m_runOffset = atoi(value.c_str());

    assert(m_configurationFile->GetKeyValue("SCENARIO", "NUM_PACKETS", value) >= 0);
    m_numPackets = atoi(value.c_str());

    assert(m_configurationFile->GetKeyValue("SCENARIO", "PACKET_LENGTH", value) >= 0);
    m_packetLength = atoi(value.c_str());

    assert (m_configurationFile->GetKeyValue ("SCENARIO", "SCENARIO_DESCRIPTION", m_scenarioFile) >= 0);
    assert (m_configurationFile->GetKeyValue ("SCENARIO", "CHANNEL_CONFIGURATION", m_channelFile) >= 0);
    assert (m_configurationFile->GetKeyValue ("SCENARIO", "STATIC_ROUTING_TABLE", m_routingFile) >= 0);

    assert (m_configurationFile->GetKeyValue ("OUTPUT", "FILE_NAME", m_fileName) >= 0);

    //Get the stack configuration from the file
    //[TRANSPORT]
    assert (m_configurationFile->GetKeyValue("STACK", "TRANSPORT_PROTOCOL", value) >= 0);

    if (!value.compare("TCP"))
    {
        m_transportProtocol = TCP_PROTOCOL;
    }
    else if (!value.compare("UDP"))
    {
        m_transportProtocol = UDP_PROTOCOL;
    }
    else
    {
    	NS_ABORT_MSG("Incorrect transport protocol " << value << " . Please fix the configuration file");
    }

    //[APPLICATION]
	assert (m_configurationFile->GetKeyValue("APPLICATION", "RATE", value) >= 0);
	m_dataRate = DataRateValue(value);

	//[NETWORK_CODING]
	assert (m_configurationFile->GetKeyValue("NETWORK_CODING", "ENABLED", value) >= 0);
	m_ncEnable = (bool) atoi(value.c_str());
	if (m_ncEnable)
	{
		m_configurationFile->GetKeyValue("NETWORK_CODING", "Q", value);
		assert (atoi(value.c_str()) >= 1 && atoi(value.c_str()) <= 6);
		m_q = (u_int8_t) atoi(value.c_str());
		m_configurationFile->GetKeyValue("NETWORK_CODING", "K", value);
		assert (atoi(value.c_str()) > 1 && atoi(value.c_str()) <= 256);
		m_k = (u_int16_t) atoi(value.c_str());
		assert (m_configurationFile->GetKeyValue("NETWORK_CODING", "RECODING", value) >= 0);
		m_recoding = bool (atoi(value.c_str()));
		assert (m_configurationFile->GetKeyValue("NETWORK_CODING", "BUFFER_TIMEOUT", value) >= 0);
		m_codingBufferTimeout = MilliSeconds(atoi(value.c_str()));
		assert (m_configurationFile->GetKeyValue("NETWORK_CODING", "OVERHEARING", value) >= 0);
		m_overhearing = bool(atoi(value.c_str()));
		assert (m_configurationFile->GetKeyValue("NETWORK_CODING", "SYSTEMATIC", value) >= 0);
		m_systematic = bool(atoi(value.c_str()));
		assert (m_configurationFile->GetKeyValue("NETWORK_CODING", "STARTRECODING", value) >= 0);
		m_startRecoding = (u_int16_t)  (atoi(value.c_str()));
	}

	//[WIFI]
	assert (m_configurationFile->GetKeyValue("WIFI", "TX_NUMBER", value) >= 0);
	m_wifiRetransmission = (atoi(value.c_str()));
	assert (m_configurationFile->GetKeyValue("WIFI", "DATA_RATE", value) >= 0);
	m_wifiDataRate = (atoi(value.c_str()));
	assert (m_configurationFile->GetKeyValue("WIFI", "CONTROL_RATE", value) >= 0);
	m_wifiControlRate = (atoi(value.c_str()));

    return true;
}

bool ConfigureScenario::ParseScenarioDescriptionFile (string confFile, string channelFile)
{
    NS_LOG_FUNCTION_NOARGS();
    string pathFile = "/src/scenario-creator/scenarios/";
    string lineString;
    fstream scenarioConfFile, scenarioChannelFile;
    //File handle variables
    int i;
    char cwdBuf [FILENAME_MAX];
    char line[256];
    u_int8_t lineNumber = 0;
    double ferValue;
    vector<double> ferVector;
    NodeDescription_t *nodeDescriptor;

    confFile = std::string(getcwd(cwdBuf, FILENAME_MAX)) + pathFile + confFile;
    channelFile = std::string(getcwd(cwdBuf, FILENAME_MAX)) + pathFile + channelFile;

    NS_LOG_DEBUG("Node description file: " << confFile << "\nChannel description file: " << channelFile);

    scenarioConfFile.open((const char *) confFile.c_str(), ios::in);
    scenarioChannelFile.open((const char *) channelFile.c_str(), ios::in);

    //Parsing the scenario description (deployment of the nodes) from the configuration file
    //File format
    //#No.	X	Y	Z	TX	RX	RT	CN
    //  1	0	0	0	 6	 0 	 0	 1

    NS_ASSERT_MSG(scenarioConfFile, "File (Channel FER file) " << channelFile << " not found: Please fix");

    while (scenarioConfFile.getline(line, 256))
    {
        lineString = string(line);
        if ((lineString.find('#') == string::npos) || (lineString.find('#') != 0)) //Ignore those lines which begins with the '#' character at its beginning
        {
            int nodeId, destNodeId, receiver, codingRouter, forwarder;
            float xCoord, yCoord, zCoord;

            sscanf(lineString.c_str(), "%d %f %f %f %d %d %d %d", &nodeId, &xCoord, &yCoord, &zCoord, &destNodeId, &receiver, &codingRouter, &forwarder);

            nodeDescriptor = new NodeDescription_t;
            nodeDescriptor->nodeId = (u_int8_t) nodeId - 1;

            nodeDescriptor->coordinates.x = xCoord;
            nodeDescriptor->coordinates.y = yCoord;
            nodeDescriptor->coordinates.z = zCoord;
            if (destNodeId)
            {
                nodeDescriptor->transmitter = true;
                nodeDescriptor->destNodeId = destNodeId - 1;
                nodeDescriptor->destinations.insert(destNodeId - 1);
            }
            else
            {
                nodeDescriptor->transmitter = false;
                nodeDescriptor->destNodeId = 0;
            }
            nodeDescriptor->receiver = receiver;
            nodeDescriptor->codingRouter = codingRouter;
            nodeDescriptor->forwarder = forwarder;
            m_nodesVector.push_back(*nodeDescriptor);
            delete nodeDescriptor;
        }
    }

    //Parsing the channel FER configuration from the file for every channel link

    NS_ASSERT_MSG(scenarioChannelFile, "File (Channel FER file) " << channelFile << " not found: Please fix");

	while (scenarioChannelFile.getline(line, 256)) {
		lineString = string(line);
		if ((lineString.find('#') == string::npos) || (lineString.find('#') != 0)) //Ignore those lines which begins with the '#' character at its beginning
		{
			for (i = 0; i < (int) lineString.size(); i++) {
				if (line [i] != ' ' && line [i] != '\t') {
					char aux[4];
					aux[0] = line[i];
					aux[1] = line[i+1];
					aux[2] = line[i+2];
 					ferValue = float(atoi(aux))/100.0;
					// The FER value must be within the interval [0,1]   --> FER = Value/100
					assert(ferValue <= 1.0);
					ferVector.push_back(ferValue);
					i = i+2;
				}
			}

			m_channelFer.insert(pair <int, vector <double> > (lineNumber, ferVector));
			lineNumber++;
			ferVector.clear();
		}
	}

    m_nodesNumber = m_nodesVector.size();

    //DEBUGGING
#ifdef NS3_LOG_ENABLE
    if (g_debug)
    {
        char message[255];
        printf("---Deployment of nodes over the scenario---\n");
        //Show nodes deployment and functionalities
        for (i = 0; i < (int) m_nodesVector.size(); i++)
        {
            sprintf(message, "Node %2d: [%3.1f, %3.1f, %3.1f]  TX:%2d  RX:%1d  RT:%1d  CN:%1d",   						\
					m_nodesVector[i].nodeId, m_nodesVector[i].coordinates.x, m_nodesVector[i].coordinates.y,            \
					m_nodesVector[i].coordinates.z, m_nodesVector[i].destNodeId, m_nodesVector[i].receiver,      		\
					m_nodesVector[i].codingRouter, m_nodesVector[i].forwarder);
            printf("%s\n", message);
        }

        //Show channel FER matrix
        channelFerIter_t iter;
        //Print the links between nodes map (matrix-shaped)
        printf("---Channel FER configuration---\n");
        for (iter = m_channelFer.begin(); iter != m_channelFer.end(); iter++)
        {
            printf("Node %2d  -  ", iter->first);
            for (i = 0; i < (int) (iter->second).size(); i++)
            {
                printf("%2f   ", (iter->second)[i]);
            }
            printf("\n");
        }
    }

#endif   //NS3_LOG_ENABLE
    scenarioConfFile.close ();
    scenarioChannelFile.close ();

    assert (m_nodesNumber == m_nodesVector.size());

    return true;
}

void ConfigureScenario::SetAttributes ()
{
	NS_LOG_FUNCTION (this);
	string value;

	//Propagation Attributes
	Config::SetDefault("ns3::RangePropagationLossModel::MaxRange", DoubleValue(20.0));
//	Config::SetDefault("ns3::RangePropagationLossModel::FirstRangeDistance", DoubleValue(20.0));
//	Config::SetDefault("ns3::RangePropagationLossModel::SecondRangeDistance", DoubleValue(40.0));

	//ARP attributes (by default, it will perpetually store the entries)
	Config::SetDefault ("ns3::ArpCache::AliveTimeout", TimeValue(Seconds(10000)));
	Config::SetDefault ("ns3::ArpCache::DeadTimeout", TimeValue(Seconds(10000)));

	//Wifi Attributes
	Config::SetDefault ("ns3::ConstantRateWifiManager::DataMode", StringValue("DsssRate11Mbps"));    //
	Config::SetDefault ("ns3::ConstantRateWifiManager::ControlMode", StringValue("DsssRate11Mbps")); //
	Config::SetDefault ("ns3::WifiMacQueue::MaxPacketNumber", UintegerValue(100000000)); 					//Maximum number of packets supported by the Wifi MAC buffer
	Config::SetDefault ("ns3::WifiMacQueue::MaxDelay", TimeValue(Seconds(10000))); //Not drop packets from buffer

	Config::SetDefault ("ns3::YansWifiPhy::TxPowerStart", DoubleValue(0.0));
	Config::SetDefault ("ns3::YansWifiPhy::TxPowerEnd", DoubleValue(0.0));
	Config::SetDefault ("ns3::YansWifiPhy::CcaMode1Threshold", DoubleValue(-150.0)); //CS power level
	 Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));    		//Disable RTS/CTS transmission
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue("DsssRate2Mbps"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200")); 		//Disable fragmentation
	Config::SetDefault ("ns3::WifiRemoteStationManager::MaxSlrc", UintegerValue(m_wifiRetransmission));		//Maximum number of transmission attempts
	Config::SetDefault ("ns3::WifiNetDevice::Mtu", UintegerValue(1500));   //

	//Transport-level attributes
	// TCP Attributes
	if (m_transportProtocol == TCP_PROTOCOL)
	{
		Config::SetDefault ("ns3::TcpSocket::TcpNoDelay", BooleanValue(true));   				//Does not seem it's working
		Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1460));
		Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(m_packetLength));
		Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(90000000));
		Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(90000000));
	}
	else
	{
		Config::SetDefault("ns3::UdpSocket::RcvBufSize", UintegerValue(90000000));
	}

	//Network Coding attribute initialization
	if (m_ncEnable)
	{
    	//NC Attributes
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::Q", UintegerValue(m_q));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::K",UintegerValue(m_k));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::Recoding", BooleanValue (m_recoding));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::BufferTimeout", TimeValue(m_codingBufferTimeout));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::OverHearing", BooleanValue(m_overhearing));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::Systematic", BooleanValue(m_systematic));
		Config::SetDefault ("ns3::IntraFlowNetworkCodingProtocol::StartRecoding", UintegerValue(m_startRecoding));
	}

	//PacketSocket buffer
	Config::SetDefault ("ns3::PacketSocket::RcvBufSize", UintegerValue(90000000));

	//Application layer
	Config::SetDefault("ns3::OnOffApplication::DataRate", DataRateValue(m_dataRate));
}

void ConfigureScenario::Init ()
{

	ParseScenarioDescriptionFile (m_scenarioFile, m_channelFile);

    NS_LOG_FUNCTION_NOARGS();
    string value;

    //	GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
    MobilityHelper mobilityHelper;
    Ptr <ListPositionAllocator> listPositionAllocator = CreateObject<ListPositionAllocator> ();

    //---------------Node creation---------------//
    m_nodeContainer.Create(m_nodesNumber);

    // Create the nodes, update the NetworkMonitor::Instance().m_nodesVector information and instance the node's mobility objects
    for (u_int16_t i = 0; i < NodeList().GetNNodes(); i++)
    {
        m_nodesVector[i].node = NodeList().GetNode(i);
        listPositionAllocator->Add(m_nodesVector[i].coordinates);
    }

    //---------------Configure the mobility of the nodes---------------//
    mobilityHelper.SetPositionAllocator(listPositionAllocator);
    mobilityHelper.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobilityHelper.Install (m_nodeContainer);
    //---------------Configure Wifi---------------// --> By the moment, this is the unique physical implementation we support
    SetWifiChannel ();

    //Set the upper layer (default configuration). Install the IP stack
    m_internetStackHelper.Install(m_nodeContainer);

    //---------------Configure Network Coding---------------//

    if (m_ncEnable)
    {
    	SetNetworkCodingLayer ();
    }

    //---------------Configure IP level---------------//
    //IP layer configuration--> 2 possibilities: If Network Coding enabled, set an IP address for each NetworkCodingNetDevice; otherwise, assign the addresses to the default WifiNetDevices
    //Note: This concrete scenario will only create a unique network for all the nodes.
    Ipv4AddressHelper ipv4AddressHelper;
    Ipv4InterfaceContainer ipv4InterfaceContainer;

    ipv4AddressHelper.SetBase ("10.0.0.0", "255.255.0.0");
    ipv4InterfaceContainer = ipv4AddressHelper.Assign (m_deviceContainer);

    //---------------Configure Routing---------------//
    ConfigureRoutingProtocol();

    //---------------Configure the application layer---------------//
    SetUpperLayer ();

    //---------------Configure Tracing---------------//
    Tracing ();

}

void ConfigureScenario::SetWifiChannel ()
{
    NS_LOG_FUNCTION(this);
    string value;

    WifiHelper wifiHelper;
    YansWifiChannelHelper channelHelper;
    YansWifiPhyHelper phyHelper;


    phyHelper = YansWifiPhyHelper::Default ();
    channelHelper.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

    NqosWifiMacHelper nQosWifiMacHelper = nQosWifiMacHelper.Default(); //By default, configure the nodes in AdHoc mode;

    // First, set the RangePropagationLossModel
    Ptr<RangePropagationLossModel> prop = CreateObject<RangePropagationLossModel > ();
    channelHelper.AddPropagationLoss("ns3::RangePropagationLossModel","MaxRange",DoubleValue(20));


    //Second, parse the scenario topology and configure the MatrixPropagationLossErrorModel
    Ptr<MatrixErrorModel> error = CreateObject<MatrixErrorModel> ();
    error->SetDefaultFer (0.0);

    ///// MatrixPropagationLossModel Configuration (taken from the channel configuration file)
    for (int i = 0; i < (int) NodeList().GetNNodes (); i++)
    {
    	for (int j = 0; j < (int) NodeList().GetNNodes (); j++)
    	{
    		if (m_channelFer.find(i) != m_channelFer.end())
    		{
    			error->SetFer (NodeList().GetNode(i)->GetId(), NodeList().GetNode(j)->GetId(), m_channelFer.find(i)->second[j]);
    		}
    		else
    		{
    			NS_LOG_ERROR("Key " << i << " not found");
    		}
    	}
    }

    phyHelper.SetErrorModel(error);

    phyHelper.SetChannel (channelHelper.Create());
    wifiHelper.Default ();
    wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211b);
    wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager");

    m_deviceContainer = wifiHelper.Install (phyHelper,
    		nQosWifiMacHelper, m_nodeContainer);
}

void ConfigureScenario::SetNetworkCodingLayer ()
{
	NS_LOG_FUNCTION_NOARGS();

	u_int16_t i;
	string value;

	NetworkCodingHelper ncHelper;

//	NS_ASSERT_MSG(m_transportProtocol == "TCP_PROTCOL","TCP and NC is not possible");

	ncHelper.Install(m_nodeContainer, "IntraFlowNetworkCodingProtocol");

	for (i = 0; i < (int) ncHelper.GetNetworkCodingList().size(); i++)
	{
		NetworkMonitor::Instance ().AddNetworkCodingLayer (ncHelper.GetNetworkCodingList()[i]);
	}
}

void ConfigureScenario::ConfigureRoutingProtocol()
{
	NS_LOG_FUNCTION(this);
	string value;
	string routingProtocol;

	//Add the static routing helper
	Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper list;
    AodvHelper aodv;
    OlsrHelper olsr;
    list.Add(staticRouting, 0);

    m_internetStackHelper.SetRoutingHelper(list);

    LoadStaticRouting(staticRouting);

}

void ConfigureScenario::LoadStaticRouting(Ipv4StaticRoutingHelper staticRouting) {
    NS_LOG_FUNCTION_NOARGS();
    fstream file;
    char cwdBuf [FILENAME_MAX];
    string fileName;
    string temp;

    //Set the path and the name of the file which contains the static routing table; afterwards, open the file
    //Grab the name of the static routing table file

    fileName = std::string(getcwd(cwdBuf, FILENAME_MAX)) + "/src/scenario-creator/scenarios/" + m_routingFile;
    file.open((const char *) fileName.c_str(), ios::in);

    //We read the file through this way because we always know the number of elements per row (5 in this static routing table)
    NS_ASSERT_MSG(file, "File " << fileName << " not found.");
    {
        char line[256];
        file.getline(line, 256);
        //Pass through the title line
        int nodeId, destination, nexthop, interface, metric;
        while (file >> nodeId >> destination >> nexthop >> interface >> metric) {
            //By default -> Two interfaces per node: 0- Loopback, 1-Output interface, that is to say, WifiNetDevice when NC layer is disabled; otherwise, will be a NetworkCodingNetDevice.
        	//Ipv4Address srcAddress = m_nodeContainer.Get((int) nodeId)->GetObject<Ipv4 > ()->GetAddress(1, 0).GetLocal();
            Ipv4Address dstAddress = m_nodeContainer.Get((int) destination)->GetObject<Ipv4 > ()->GetAddress(1, 0).GetLocal();
            Ipv4Address nextHopAddress = m_nodeContainer.Get((int) nexthop)->GetObject<Ipv4 > ()->GetAddress(1, 0).GetLocal();

            NS_LOG_DEBUG("Node " << (int) nodeId << " IP address " <<
                    m_nodeContainer.Get((int) nodeId)->GetObject<Ipv4 > ()->GetAddress(1, 0).GetLocal()
                    << " Dest adress " << dstAddress << " Nexthop adress " << nextHopAddress);
            Ptr<Ipv4StaticRouting> routingEntry = staticRouting.GetStaticRouting(m_nodeContainer.Get((int) nodeId)->GetObject<Ipv4 > ());

            routingEntry->AddHostRouteTo(dstAddress, nextHopAddress, interface, metric);
        }
    }
    file.close();

    string value;

	assert (m_configurationFile->GetKeyValue("OUTPUT", "ROUTING_TABLES", value) >= 0);
	if (atoi(value.c_str()))
	{

		string fileName = "traces/route";

		Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper > (fileName, std::ios::out);
		staticRouting.PrintRoutingTableAllEvery (Seconds(5), routingStream);
	}
}

std::string ConfigureScenario::CheckTransportLayer()
{
    NS_LOG_FUNCTION(this);
    if (m_transportProtocol == TCP_PROTOCOL)
        return "ns3::TcpSocketFactory";
    else if (m_transportProtocol == UDP_PROTOCOL)
        return "ns3::UdpSocketFactory";
    else
        return "ns3::TcpSocketFactory";
}

void ConfigureScenario::SetUpperLayer()
{

    NS_LOG_FUNCTION(this);
    string value;
    u_int16_t i;
    u_int16_t portBase = 50000;
    u_int16_t portOffset = 0;
    multiset<u_int16_t>::iterator iter;
    pair<multiset<u_int16_t>::iterator, multiset<u_int16_t>::iterator> destinationsList;

    //Application definition (for this concrete testbed, we are going to use the OnOffApplication environment to inject the traffic into the scenario
    //Transmission nodes --> We have to parse the NetworkMonitor::Instance().m_nodesVector looking for the nodes' configuration
    //Reception nodes --> The same method as for the transmitters
//    double offset = 0;

    //Read the number of packets
    if (! m_propTracing->GetTraceInfo().numPackets)
    {
    	m_propTracing->GetTraceInfo().numPackets = m_numPackets;
    }

    //Before starting the transmission, send dummy packets in order to fill, by means of UdpEcho applications, the ARP cache
    double offset = 0;
    ApplicationContainer clientApps, serverApps;
    for (i = 0; i < (int) m_nodesVector.size(); i++) {
        UdpEchoServerHelper echoServer(9);
        serverApps = echoServer.Install(m_nodesVector[i].node);
        serverApps.Start(Seconds(4.0));
        serverApps.Stop(Seconds(20.0));

        for (int j = 0; j < (int) m_nodesVector.size(); j++)
        {
            if (i != j)
            {
                UdpEchoClientHelper echoClient(m_nodesVector[j].node->GetObject<Ipv4 > ()->GetAddress(1, 0).GetLocal(), 9);
                echoClient.SetAttribute("MaxPackets", UintegerValue(1));
                echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
                echoClient.SetAttribute("PacketSize", UintegerValue(200));

                clientApps = echoClient.Install(m_nodesVector[i].node);
                clientApps.Start(Seconds(1.0 + offset));
                clientApps.Stop(Seconds(20.0));

                offset += 0.01;
            }
        }
    }
    ///// End of cache table filling
    ApplicationContainer sourceAppContainer;
    ApplicationContainer sinkAppContainer;

//    double offset =< 0;
    for (i = 0; i < (int) m_nodesVector.size(); i++) {
        //Configuring the application layer
        if (m_nodesVector[i].transmitter) {
            for (iter = m_nodesVector[i].destinations.begin(); iter != m_nodesVector[i].destinations.end(); iter++, portOffset++) {
                //One pair Application/Sink for each one
                //Application (OnOffApplication)
                OnOffHelper onOff (CheckTransportLayer(), Address(InetSocketAddress((m_nodesVector[*iter].node->GetObject<Ipv4 > ())->GetAddress(1, 0).GetLocal(), portBase + portOffset)));
                //onOff.SetAttribute("OnTime", RandomVariableValue(ConstantVariable(1)));
                onOff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));  //Change for ns-3.20
                onOff.SetAttribute("OffTime",StringValue ("ns3::ConstantRandomVariable[Constant=0]")); //Change for ns-3.20
                onOff.SetAttribute("PacketSize", UintegerValue(m_packetLength));
                onOff.SetAttribute("MaxBytes", UintegerValue(m_packetLength * m_numPackets));
                sourceAppContainer = onOff.Install (NodeList().GetNode(i));
                sourceAppContainer.Start(Seconds(20.0));
                sourceAppContainer.Stop(Seconds(10000.0));
                NetworkMonitor::Instance().GetSourceApps().Add (sourceAppContainer);
                NS_LOG_DEBUG("OnOff Destination: " << (m_nodesVector [*iter].node->GetObject<Ipv4 > ())->GetAddress(1, 0).GetLocal() << " Port: " << portBase + portOffset);

                //Packet sink
                PacketSinkHelper packetSink (CheckTransportLayer(), Address(InetSocketAddress (Ipv4Address::GetAny(), portBase + portOffset)));
                sinkAppContainer = packetSink.Install(NodeList().GetNode(*iter));
                sinkAppContainer.Start(Seconds(1.0));
                sinkAppContainer.Stop(Seconds(10000.0));
                NetworkMonitor::Instance().GetSinkApps().Add (sinkAppContainer);
                NS_LOG_DEBUG ("Sink defined at IP " << (m_nodesVector[*iter].node->GetObject<Ipv4 > ())->GetAddress(1, 0).GetLocal() << " , listening at port " << portBase + portOffset);
            }
        }
    }

}


void ConfigureScenario::Tracing ()
{
	NS_LOG_FUNCTION (this);
	string value;

    Ptr<ProprietaryTracing> prop = CreateObject<ProprietaryTracing > ();

    prop->GetTraceInfo().fileName = m_fileName;
    prop->GetTraceInfo().totalRuns = m_totalRuns;
    prop->GetTraceInfo().packetLength = m_packetLength;
    prop->GetTraceInfo().numPackets = m_numPackets;

	//Application level tracing
	assert (m_configurationFile->GetKeyValue("OUTPUT", "APPLICATION_LEVEL_SHORT_TRACING", value) >= 0);
	if (atoi(value.c_str()))
	{
		m_propTracing->EnableApplicationShortTraceFile ();
	}

	//Network Coding level tracing
	if (m_ncEnable)
	{
		assert (m_configurationFile->GetKeyValue("OUTPUT", "NETWORK_CODING_SHORT_TRACING", value) >= 0);
		if (atoi(value.c_str()))
		{
			m_propTracing->EnableNCShortTraceFile();
		}
	}

	//Phy (Wifi) level tracing
	assert (m_configurationFile->GetKeyValue("OUTPUT", "PHY_WIFI_SHORT_TRACING", value) >= 0);
		if (atoi(value.c_str()))
		{
			m_propTracing->EnableWifiPhyLevelShortTracing ();
		}
}
