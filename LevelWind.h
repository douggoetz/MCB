/*
 *  LevelWind.h
 *  Definition of a class to control the level wind
 *  Author: Alex St. Clair
 *  November 2018
 *
 *  This file defines an Arduino library (C++ class) that controls
 *  the level wind. It inherits from the Technosoft class, which
 *  implements communication with the Technosoft motor controllers.
 */

#ifndef LEVEL_WIND_H
#define LEVEL_WIND_H

#include "Arduino.h"
#include "HardwareSerial.h"
#include "WProgram.h"
#include "Technosoft.h"
#include "TML_Instructions_Addresses.h"
#include <stdint.h>

#define LEVEL_WIND_AXIS		2

#define LW_STEPS_PER_MM		10500

#define LW_HOME_MILLIS		30000

// SET THIS MACRO TO CHANGE THE CAM RATIO
#define LW_MM_PER_ROT		1 // mm of LW linear motion per reel rotation

#define LW_MM_PER_SEC		9.8304 // mechanical constant, converts 1 mm/s to internal units
#define RPM_PER_SEC			(1.0/60.0) // rpm to rps
#define LW_SPEED_CONV		(LW_MM_PER_ROT*LW_MM_PER_SEC*RPM_PER_SEC) // multiply by reel speed to get LW speed

/// Level wind addresses for MCB-SOLO configuration FLight_System_V6_RACHuTS_EmCamVariable
//#ifdef INST_RACHUTS // defined in HardwareMCB.h
//#define STOP_PROFILE_LW		0x4025
//#define WIND_OUT_LW			0x4028
//#define WIND_IN_LW			0x404C
//#define SET_CENTER_LW		0x4060
//#define HOME_LW				0x4077
//#endif

#ifdef INST_RACHUTS // defined in HardwareMCB.h
#define STOP_PROFILE_LW		0x4022
#define WIND_OUT_LW			0x4025
#define WIND_IN_LW			0x4050
#define SET_CENTER_LW		0x4067
#define HOME_LW				0x407D
#endif

//////Level wind addresses for old EBOX system (non-mondo)
//#ifdef INST_FLOATS // defined in HardwareMCB.h
//#define STOP_PROFILE_LW		0x4025
//#define WIND_OUT_LW			0x4028
//#define WIND_IN_LW			0x404C
//#define SET_CENTER_LW		0x4060
//#define HOME_LW				0x4077
//#endif

//Level wind addresses for MONDO
#ifdef INST_FLOATS // defined in HardwareMCB.h
#define STOP_PROFILE_LW		0x4022
#define WIND_OUT_LW			0x4025
#define WIND_IN_LW			0x4050
#define SET_CENTER_LW		0x4067
#define HOME_LW				0x407D
#endif

class LevelWind : public Technosoft {
public:
	LevelWind(uint8_t expeditor_axis);
	bool StopProfile();
	bool SetCenter();
	bool Home();
	bool UpdatePosition();
	bool WindOut(float reel_speed);
	bool WindIn(float reel_speed);

	float absolute_position; // in mm, relative to home

private:
	bool camming;
};

#endif