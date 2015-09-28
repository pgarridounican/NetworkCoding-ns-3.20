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
 *         Ramón Agüero Calvo <ramon@tlmat.unican.es>
 */

#ifndef CONFIGURE_SCENARIO_H_
#define CONFIGURE_SCENARIO_H_

#include "ns3/node-list.h"

#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/propagation-loss-model.h"

#include "ns3/olsr-helper.h"
#include "ns3/aodv-helper.h"
#include "ns3/ipv4-static-routing-helper.h"

#include "ns3/inet-socket-address.h"

//Proprietary loss propagation models
#include "ns3/error-model.h"

#include <fstream>
#include <map>
#include <set>
#include <unistd.h>

//Scenario setup related files
#include "ns3/proprietary-tracing.h"
#include "ns3/configuration-file.h"

#include "ns3/network-monitor.h"

namespace ns3 {

//Data types used to handle the configuration file-based information

enum TransportProtocol_t
{
	TCP_PROTOCOL,
	UDP_PROTOCOL
};

typedef struct {
	Ptr<Node> node;				//Pointer to the corresponding node
	u_int8_t nodeId;			//Node's identity
	Vector coordinates;			//Node location
	bool transmitter;			//Is the node a transmitter?
	u_int8_t destNodeId;		//If so, specify the destination node's ID. Further version: Multiple flows per source (maybe a map which contains all the possible destinations?)
	std::multiset <u_int16_t> destinations;
	bool receiver;				//Is the node a receiver?
	bool codingRouter;				//Enabled when the node only acts as an intermediate router, without implementing the NC layer within its behaviour
	bool forwarder;	//Enabled if the node has got the NC architecture
	uint8_t nextHope;

} NodeDescription_t;


//class ProprietaryTracing;

/**
 * \defgroup configurescenario Configure Scenario
 * \brief Scenario description helper for a wireless mesh network.
 */


class ConfigureScenario	: public Object
{
public:
	/**
	 * Attribute handler
	 */
	static TypeId GetTypeId (void);
	/**
	 * Default constructor
	 */
	ConfigureScenario();
	/**
	 * Default destructor
	 */
	~ConfigureScenario();
	/**
	 * \brief Method that reads the configuration file and sets the corresponding parameters. NOTE: These config files MUST be stored in
	 * \param confFile Configuration file (Raw name, without the cfg extension)
	 * \return True if successfully read the configuration file; false otherwise
	 */
	bool ParseConfigurationFile (std::string confFile);
	/**
	 * \brief Key method that receives both nodes and links configuration and outlines the corresponding scenario. NOTE: Only when the scenario is defined by an extern file
	 * \param confFile File which contains the description of the deployment of the nodes (i.e. number, location and functionalities)
	 * \param channelFile  File which represent the FER between each two pair of nodes (it has the shape of a m_nodesNumber x m_nodesNumber matrix)
	 * \return True if successfully read the two configuration files; false otherwise
	 */
	bool ParseScenarioDescriptionFile (std::string confFile, std::string channelFile);

	/**
	 * \brief Parse the channel description (Propagation and error models)
	 */
	void ParseChannelDescriptionFile ();
	/**
	 * \brief With the given parameters, generate a random wireless scenario where all the nodes are within a rectangle-shaped area
	 */
	void GenerateRandomScenario ();

	/**
	 * \brief Generate a naive line topology defined by the following input values. NOTE: This first implementation will automatically assign
	 * 		  the transmitter and receiver roles to the first and last nodes, respectively
	 */
	void GenerateLineTopology ();

	/**
	 *  \brief Member function in charge of the initialization of all the system attributes.
	 */
	void SetAttributes ();
	/**
	 * \brief Initializer method (create containers, Wifi Adhoc Network, IP addresses, applications, etc.)
	 */
	void Init ();
	/**
	 * \brief Configure the WiFi physical layer, which embraces modulation, rates, propagation issues and the error models
	 * that will define the behavior of the channel (i.e. simple error rate-based model, BER-based formulae, Hidden Markov
	 * Process or BEAR wireless channel)
	 */
	void SetWifiChannel ();
	/**
	 * \brief Configure all the stuff regarding the setup of the network coding layer, in case it is used.
	 */
	void SetNetworkCodingLayer ();
	/**
	 * Setup the routing protocol according to the value held by the m_routingProtocol variable
	 */
	void ConfigureRoutingProtocol ();
	/**
	 * Load a file which contains the configuration (table) of a fully-functional static routing scheme
	 */
	void LoadStaticRouting (Ipv4StaticRoutingHelper staticRouting);
	/**
	 * \brief Simple function that sets the corresponding parameters according to the transport layer chosen
	 * \return A string which will be used to directly define the application; that is to say
	 */
	std::string CheckTransportLayer();
	/**
	 * \brief Method used to define the upper layer setup (TCP/UDP) and so on
	 */
	void SetUpperLayer ();

	/**
	 * Prepare all the stuff related to the tracing system (i.e. enablers, fileNames, etc.)
	 * It is worth highlighting that this member function is tightly linked to the ProprietaryTracing class
	 */
	void Tracing ();

	/**
	 * \return The transport protocol (EnumValue  TransportProtocol_t)
	 */
	inline TransportProtocol_t GetTransportProtocol() {return m_transportProtocol;}
	/**
	 * \param transportProtocol The transport protocol (EnumValue  TransportProtocol_t)
	 */
	inline void SetTransportProtocol (TransportProtocol_t transportProtocol) {m_transportProtocol = transportProtocol;}
	/**
	 * \return A pointer to the ProprietaryTracing object
	 */
	inline Ptr<ProprietaryTracing> GetProprietaryTracing () {return m_propTracing;}


//private:
	uint32_t m_totalRuns;
	uint32_t m_runOffset;
	uint32_t m_packetLength;
	u_int32_t m_numPackets;
	string m_routingFile;
	uint32_t m_nodesNumber;

	string m_fileName;

	TransportProtocol_t m_transportProtocol;				  //TCP_PROTOCOL or UDP_PROTOCOL
	DataRateValue m_dataRate;

	bool m_ncEnable;
	Time m_codingBufferTimeout;
	u_int8_t m_q;
	u_int16_t m_k;
	bool m_recoding;
	bool m_overhearing;
	bool m_systematic;
	uint16_t m_startRecoding;


	u_int32_t m_wifiRetransmission;
	string m_wifiDataRate;
	string m_wifiControlRate;

	string m_scenarioFile;
	string m_channelFile;

	//Connection to other classes
	Ptr <ProprietaryTracing> m_propTracing;					  //Object to handle the proprietary tracing environment
	Ptr <ConfigurationFile> m_configurationFile;			  //Read the file that contains the values of the parameters that define the scenario behavior (i.e. number of packets, FER...)

	vector<NodeDescription_t> m_nodesVector;

	//Needed variables, helpers or containers
	NodeContainer m_nodeContainer;
	NetDeviceContainer m_deviceContainer;
	InternetStackHelper m_internetStackHelper;

	//Container of the link FER matrix (read from the channel FER configuration file)
	typedef map <int, vector<double> > channelFer_t;
	typedef map <int, vector<double> >::const_iterator channelFerIter_t;
	channelFer_t m_channelFer;

};


} //End namespace ns3
#endif /* CONFIGURE_SCENARIO_H_ */
