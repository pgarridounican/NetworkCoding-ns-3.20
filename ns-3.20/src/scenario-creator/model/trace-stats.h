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
 * Author: David Gómez Fernández <dgomez@tlmat.unican.es>
 *		   Ramón Agüero Calvo <ramon@tlmat.unican.es>
 */

#include <vector>


#ifndef TRACE_STATS_H_
#define TRACE_STATS_H_


namespace ns3 {


typedef struct {

	u_int32_t rxFrames;
	u_int32_t rxBytes;

	u_int32_t collisions;					//Collision counter
	std::vector<u_int8_t> traces;			//Vector that contains '1' if the frame reception was successful and '0' otherwise

} WifiStats_t;


typedef struct {
	u_int32_t dupAcks;
	u_int32_t tripleDupAcks;
	u_int32_t rtoRetx;

} TcpStats;

typedef struct {
	u_int32_t txPackets;
	u_int32_t txBytes;

	u_int32_t rxPackets;
	u_int32_t rxBytes;

	std::vector<double> txTimestamp;
	std::vector<double> rxTimestamp;

} ApplicationLayerStats_t;

typedef struct {

	bool forwarder;
	bool source;
	bool recode;
	uint8_t k;
	uint8_t q;

	uint32_t packetLenght;

	u_int32_t txPackets;
	u_int32_t txBytes;

	u_int32_t rxPackets;
	uint32_t rxDiscacerdedPackets;
	u_int32_t rxBytes;

	std::vector<double> txTimestamp;
	std::vector<double> rxTimestamp;

} NetworkCodingLayerStats_t;

typedef struct {

//	bool forwarder;
//	bool source;
//	bool recode;
//	uint8_t k;
//	uint8_t q;

	uint32_t packetLenght;

	u_int32_t txPackets;
	u_int32_t rxPackets;
	u_int32_t rxEndPackets;
	uint32_t rxErrorPackets;
	uint32_t rxCollisionPackets;
	u_int32_t rxDropPackets;


//	std::vector<double> txTimestamp;
//	std::vector<double> rxTimestamp;

} WifiPhyLayerStats_t;

} //End namespace ns3

#endif /* TRACE_STATS_H_ */
