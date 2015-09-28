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

#ifndef PROPRIETARY_TRACING_H_
#define PROPRIETARY_TRACING_H_

#include "ns3/object.h"
#include "ns3/random-variable.h"
#include "ns3/traced-value.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "ns3/simulator.h"
#include "ns3/packet.h"
#include "ns3/address.h"

#include "ns3/core-module.h"
#include "ns3/wifi-module.h"
#include "ns3/network-coding-module.h"

#include "ns3/flow-monitor-module.h"

#include "network-monitor.h"


#include "trace-stats.h"

#include <math.h>

namespace ns3
{

struct TracingInformation
{
	TracingInformation();
	~TracingInformation();
	//Simulation setup
	u_int32_t run;
	u_int32_t runOffset;
	u_int32_t totalRuns;
	u_int16_t packetLength;
	u_int16_t numPackets;
	Time applicationStartTime;

	double fer;
	double fer2;
	std::string fileName;
};

class ProprietaryTracing: public Object
{
public:
	/**
	 * Default constructor
	 */
	ProprietaryTracing();
	/**
	 * Default destructor
	 */
	~ProprietaryTracing();
	/*
	 *  Makes the trace information container public (connection to the ConfigureScenario class)
	 *  \returns A reference to the struct that holds the information about the tracing system
	 */
		inline struct TracingInformation& GetTraceInfo () {return m_traceInfo;}
	/*
	 * Take care of printing all the statistics upon the ending of a simulation
	 */
	void PrintStatistics ();

	/*
	 * Enable and open the trace file corresponding to the Application layer (short format)
	 */
	void EnableApplicationShortTraceFile ();

	/*
	 * Print the final statistics gathered at the application layer
	 */
	void PrintApplicationStatistics ();

	/*
	 * Enable and open the trace file corresponding to the Application layer (short format)
	 */
	void EnableNCShortTraceFile ();

	/*
	 * Print the final statistics gathered at the application layer
	 */
	void PrintNCStatistics ();
	/*
	 * Enable and open the trace file corresponding to the Application layer (short format)
	 */
	void EnableWifiPhyLevelShortTracing ();

	/*
	 * Print the final statistics gathered at the application layer
	 */
	void PrintWifiPhyStatistics ();
	/*
	 * Handle the traces off packet at wifi Level
	 */
	void WifiTxBeginEvent (Ptr<const Packet> packet);
	/*
	 * Handle the traces off packet at wifi Level
	 */
	void WifiTxDropEvent (Ptr<const Packet> packet);
	/*
	 * Handle the traces off packet at wifi Level
	 */
	void WifiTxEndEvent (Ptr<const Packet> packet);
	/*
	 * Handle the traces off packet at wifi Level
	 */
	void WifiRxBeginEvent (Ptr<const Packet> packet);
	/*
	 * Handle the traces off packet at wifi Level
	 */
	void WifiRxDropEvent (Ptr<const Packet> packet);
	/*
	 * Handle the traces off packet at wifi Level
	 */
	void WifiRxEndEvent (Ptr<const Packet> packet);

private:
	//File name
	TracingInformation m_traceInfo;

	//File handlers
	//Application level tracing
	fstream m_applicationLevelShortTraceFile;

	//NC level tracing
	fstream m_ncLevelShortTraceFile;

	//Phy level tracing
	fstream m_phyWifiLevelShortTracingFile;
};

}  //End namespace ns3

#endif /* PROPRIETARY_TRACING_H_ */
