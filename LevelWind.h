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

#ifdef INST_RACHUTS // defined in HardwareMCB.h
#define STOP_PROFILE_LW		0x4022
#define START_CAMMING_LW	0x4025
#define SET_CENTER_LW		0x403F
#define HOME_LW				0x4056
#endif
#ifdef INST_FLOATS // defined in HardwareMCB.h
#define STOP_PROFILE_LW		0x401F
#define WIND_OUT_LW			0x4022
#define WIND_IN_LW			0x404D
#define SET_CENTER_LW		0x4064
#define HOME_LW				0x407B
#endif

class LevelWind : public Technosoft {
public:
	LevelWind(uint8_t expeditor_axis);
	bool StopProfile();
	bool SetCenter();
	bool Home();
	bool UpdatePosition();

#ifdef INST_RACHUTS
	bool StartCamming();
#endif

#ifdef INST_FLOATS
	bool WindOut();
	bool WindIn();
#endif

	float absolute_position; // in mm, relative to home
private:
	bool camming;
};

#endif