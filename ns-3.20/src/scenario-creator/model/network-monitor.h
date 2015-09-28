/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Universidad de Cantabria
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
 * Author: Luis Francisco Diez Fernandez <ldiez@tlmat.unican.es>
 * 		   David Gómez Fernández <dgomez@tlmat.unican.es>
 *		   Ramón Agüero Calvo <ramon@tlmat.unican.es>
 */

#ifndef NETWORK_MONITOR_H_
#define NETWORK_MONITOR_H_

#include <map>
#include <iostream>

#include "ns3/core-module.h"
#include "ns3/application-container.h"
#include "ns3/ipv4-address.h"
#include "trace-stats.h"
#include <stdio.h>
#include <unistd.h>
#include <iostream>

#include "ns3/network-coding-l4-protocol.h"



namespace ns3 {

//Struct which contains all the information relative to the nodes (i.e. object, ID, ubication, behaviour)
//typedef struct {
//	Ptr<Node> node;				//Pointer to the corresponding node
//	u_int8_t nodeId;			//Node's identity
//	Vector coordinates;			//Node location
//	bool transmitter;			//Is the node a transmitter?
//	u_int8_t destNodeId;		//If so, specify the destination node's ID. Further version: Multiple flows per source (maybe a map which contains all the possible destinations?)
//	std::multiset <u_int16_t> destinations;
//	bool receiver;				//Is the node a receiver?
//	bool codingRouter;				//Enabled when the node only acts as an intermediate router, without implementing the NC layer within its behaviour
//	bool forwarder;	//Enabled if the node has got the NC architecture
//	uint8_t nextHope;
//
//} NodeDescription_t;

	class NetworkMonitor: public Object
	{
		friend class ConfigureScenario;
		friend class ProprietaryTracing;
	public:
		static TypeId GetTypeId ( void );
		static NetworkMonitor& Instance ( );
		/*
		 * Get the number of elements stored in the NC vector
		 * \returns The NetworkCodingL4Protocol vector size
		 */
		size_t GetNetworkCodingVectorSize ();
		/**
		 * Get the i-th element stored in the Network Coding vector
		 * \param i Index of the desired element
		 */
		Ptr<NetworkCodingL4Protocol> GetNetworkCodingElement (u_int32_t i);
		/**
		 * Add a NetworkCodingL4Protocol object to the monitor's vector
		 * \param element A pointer to the object
		 */
		void AddNetworkCodingLayer (Ptr <NetworkCodingL4Protocol> element);

		/*
		 * \return The application container that holds the source apps
		 */
		inline ApplicationContainer& GetSourceApps () {return m_sourceApps;}

		/*
		 * \return The application container that holds the source apps
		 */
		inline ApplicationContainer& GetSinkApps () {return m_sinkApps;}
		/**
		 * Manually remove the information stored in this class (i.e. vectors, etc.)
		 */
		void Reset ();

//		vector<NodeDescription_t> m_nodesVector;

	private:
		NetworkMonitor ( );
		~NetworkMonitor ( );

		inline friend std::ostream& operator << (std::ostream& out, const NetworkMonitor& rm);

		//Application Layer monitoring
		ApplicationContainer m_sourceApps;
		ApplicationContainer m_sinkApps;

		//Network Coding Layer monitoring
		std::vector<Ptr <NetworkCodingL4Protocol> > m_networkCodingVector;  //Pointers to the base class: recall to dynamic cast!
	};
}  //namespace ns3

#endif /* NETWORK_MONITOR_H_ */
