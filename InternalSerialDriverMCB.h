/*
 *  InternalSerialDriverMCB.cpp
 *  File defining the serial driver for communicating with the
 *	main payload computer (DIB) from the MCB
 *  Author: Alex St. Clair
 *  February 2018
 */

#ifndef INTERNALSERIALDRIVERMCB_H_
#define INTERNALSERIALDRIVERMCB_H_

#include "StorageManagerMCB.h"
#include "ActionsMCB.h"
#include "HardwareMCB.h"
#include "MCBComm.h"
#include "Queue.h"
#include <StdInt.h>

// TODO: implement struct to hold values from DIB/PIB
struct MCBParameters_t {
	// deploy
	float deploy_length;
	float deploy_velocity;
	float deploy_acceleration;

	// retract
	float retract_length;
	float retract_velocity;
	float retract_acceleration;

	// dock
	float dock_length;
	float dock_velocity;
	float dock_acceleration;
};

class InternalSerialDriverMCB {
public:
	InternalSerialDriverMCB(Queue * state_q, Queue * monitor_q);
	~InternalSerialDriverMCB(void) { };

	void RunDriver(void);

	MCBComm dibComm;

	MCBParameters_t mcbParameters = {0};

private:
	void HandleASCII(void);

	// Interface objects
	Queue * state_queue;
	Queue * monitor_queue;

	StorageManagerMCB storageManager;
};

#endif