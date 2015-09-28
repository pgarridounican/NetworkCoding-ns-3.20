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
 *		   Ramón Agüero Calvo <ramon@tlmat.unican.es>
 */

#include <sys/types.h>
#include <assert.h>

#include "scratch-logging.h"
#include "ns3/scenario-creator-module.h"
#include "ns3/simulation-singleton.h"


using namespace ns3;
using namespace std;

u_int32_t GetNumberOfSimulations(string fileName);
uint32_t GetSimulationOffset(string fileName);

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
    char output [255];
    string traceFile;
    u_int32_t runs = 0;
    u_int32_t runOffset = 0;

    //Default variables  --> Available scenarios: two-nodes, x and butterfly (the last two scenarios present as well three different error location policies)
    //Configuration file
    string configuration = "network-coding-scenario";

    //Scenario setup variables
    Ptr <ProprietaryTracing> propTracing;

    //Random seed related values
    u_int32_t runCounter;

    //Activate the logging  (from the library scratch-logging.h, just modify there those LOGGERS as wanted)
    EnableLogging();

    runs = GetNumberOfSimulations(configuration);
    runOffset = GetSimulationOffset(configuration);

    for (runCounter = runOffset; runCounter <= runs; runCounter++) {

    	begin = clock();

    	SeedManager::SetRun(runCounter);
    	SeedManager::SetSeed(runCounter+1);

    	//Create the scenario (auto-configured by the ConfigureScenario object)
		SimulationSingleton <ConfigureScenario>::Get ()->ParseConfigurationFile (configuration);

        SimulationSingleton <ConfigureScenario>::Get ()->SetAttributes ();

    	SimulationSingleton <ConfigureScenario>::Get ()->Init ();

    	propTracing = SimulationSingleton <ConfigureScenario>::Get ()->GetProprietaryTracing ();
    	propTracing->GetTraceInfo().run = runCounter;

    	//Run the simulation
    	Simulator::Stop(Seconds (1000.0));
    	Simulator::Run ();

    	end = clock ();
    	//Print final statistics
		sprintf(output, "[%04.5f sec] - Run %d ", (double) (end - begin) / CLOCKS_PER_SEC,
				runCounter);
		printf("%s\n", output);

		SimulationSingleton <ConfigureScenario>::Get ()->m_propTracing->PrintStatistics();
		NetworkMonitor::Instance().Reset ();
    	Simulator::Destroy ();
    }
    return 0;
} //end main

/**
 * Read the configuration file in order to get the number of simulations to create the main loop
 */
u_int32_t GetNumberOfSimulations(string fileName)
{
	ConfigurationFile config;
	string temp;
	config.LoadConfig (config.SetConfigFileName ("/src/scenario-creator/config/", fileName));
	config.GetKeyValue ("SCENARIO", "RUN", temp);
    return  (u_int32_t) atoi(temp.c_str());
}

uint32_t GetSimulationOffset(string fileName)
{
	ConfigurationFile config;
	string temp;
	config.LoadConfig (config.SetConfigFileName ("/src/scenario-creator/config/", fileName));
	config.GetKeyValue ("SCENARIO", "RUN_OFFSET", temp);
    return  (u_int32_t) atoi(temp.c_str());
}
