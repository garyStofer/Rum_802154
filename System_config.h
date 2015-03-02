/*
 * configure.h
 *
 *  Created on: Feb 3, 2015
 *      Author: gary
 */

#ifndef SYSTEM_CONFIG_H_
#define SYSTEM_CONFIG_H_

#ifndef NODE_SLEEP
#define NODE_SLEEP 1	// Allows the ENDNODES to Sleep
#endif
#define DEMO 0			// Do not set  this -- Invokes different association according to Signal strength only

#ifndef DEBUG
#define DEBUG 0			// This and the Serial define enable UART debug messages to be sent
#endif
#ifndef SERIAL
#define SERIAL 0
#endif
#define PAN_CHANNEL 0xff// Setting this to 0xff indicates that the coordinator does a channel sweep and decides which CH to use

#define STORED_FRAMES 6  // Number of stored frames the COORD and Routers can keep for Sleeping nodes. Ideally one per sleeping end node

// Two defines for my hardware platforms
#define MINER_A   11
#define MINER_B   12
#define PLATFORM MINER_B


#endif /* SYSTEM_CONFIG_H_ */
