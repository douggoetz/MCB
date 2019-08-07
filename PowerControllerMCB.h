/*
 *  PowerControllerMCB.h
 *  File the power controller for the motor control board
 *  Author: Alex St. Clair
 *  February 2018
 */

#ifndef POWERCONTROLLERMCB_H_
#define POWERCONTROLLERMCB_H_

#include "Arduino.h"
#include "WProgram.h"
#include "HardwareMCB.h"
#include <StdInt.h>
#include <StdLib.h>

class PowerControllerMCB {
public:
	PowerControllerMCB();
	~PowerControllerMCB() { };
	
	void ReelOn();
	void LevelWindOn(); // will also turn on the reel if off

	void ReelOff(); // will also turn off the lw if on
	void LevelWindOff();
};

#endif