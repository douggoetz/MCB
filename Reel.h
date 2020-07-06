/*
 *  Reel.h
 *  Definition of a class to control the reel
 *  Author: Alex St. Clair
 *  January 2018
 *
 *  This file defines an Arduino library (C++ class) that controls
 *  the reel. It inherits from the Technosoft class, which implements
 *  communication with the Technosoft motor controllers.
 *
 *  Units:
 *  - Revolutions: rot
 *  - Speed: rpm
 *  - Acc: rot/s^2
 */

#ifndef REEL_H
#define REEL_H

#include "Arduino.h"
#include "HardwareSerial.h"
#include "WProgram.h"
#include "Technosoft.h"
#include "TML_Instructions_Addresses.h"
#include "StorageManagerMCB.h"
#include <stdint.h>

// general info
#define REEL_AXIS			1
#define MAX_REVOLUTIONS		8000.0	// rot

// conversion factors that depend on the instrument
#ifdef INST_RACHUTS // defined in HardwareMCB.h
#define REEL_UNITS_PER_REV	24000.0
#define SPEED_CONVERSION	0.32
#define DEFAULT_FULL_SPEED	250.0	// rpm
#define DEFAULT_DOCK_SPEED	80.0	// rpm
#define ACC_CONVERSION		0.01537
#define DEFAULT_ACC			1.0		// rot/s^2
#define MAX_ACC				2.0		// rot/s^2
#define MAX_SPEED			400.0 	// rpm
#endif

#ifdef INST_FLOATS // defined in HardwareMCB.h
#define REEL_UNITS_PER_REV	344.0
#define SPEED_CONVERSION	0.00459 // iu/rpm
#define DEFAULT_FULL_SPEED	60.0	// rpm
#define DEFAULT_DOCK_SPEED	60.0	// rpm
#define ACC_CONVERSION		0.000035
#define DEFAULT_ACC			20.0	// rot/s^2
#define MAX_ACC				25.0	// rot/s^2
#define MAX_SPEED			120.0 	// rpm
#endif

// Technosoft function addresses
#ifdef INST_RACHUTS // defined in HardwareMCB.h
#define STOP_PROFILE_R		0x402C
#define REEL_VARIABLE_R		0x4031
#define CAM_SETUP_R			0x4043
#define CAM_STOP_R			0x4051
#define BRAKE_ON_R			0x4057
#define BRAKE_OFF_R			0x405C
#endif

#ifdef INST_FLOATS // defined in HardwareMCB.h
#define STOP_PROFILE_R		0x4026
#define REEL_VARIABLE_R		0x402B
#define CAM_SETUP_R			0x4046
#define CAM_STOP_R			0x4055
#define BRAKE_ON_R			0x405A
#define BRAKE_OFF_R			0x405F
#endif

// Position logging in case of power cycle
#define POS_LOG_FILE		FSW_DIR "pos_log.dat"

class Reel : public Technosoft {
public:
	Reel(uint8_t expeditor_axis);
	bool SetPosition(float new_pos);
	bool SetPosition(int32_t new_pos);
	bool UpdatePosition();
	bool UpdateSpeed();
	void SetToStoredPosition();
	bool StopProfile();
	bool ReelIn(float num_revolutions, float speed, float acc=DEFAULT_ACC);
	bool ReelOut(float num_revolutions, float speed, float acc=DEFAULT_ACC);
	bool CamSetup();
	bool CamStop();
	bool BrakeOn();
	bool BrakeOff();
	void EmergencyStop();
	void RemoveEmergencyStop();

	int32_t absolute_position; // in encoder units (24000 per rotation)
	float speed;
private:
	StorageManagerMCB storageManager; // supports multiple objects
};

#endif