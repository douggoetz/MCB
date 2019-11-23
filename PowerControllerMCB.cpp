/*
 *  PowerControllerMCB.cpp
 *  File implementing the MCB power controller
 *  Author: Alex St. Clair
 *  February 2018
 */

#include "PowerControllerMCB.h"

PowerControllerMCB::PowerControllerMCB()
{
	// pin setup and disable level wind at startup
	pinMode(MC2_ENABLE_PIN, OUTPUT);
	pinMode(MTR2_ENABLE_PIN, OUTPUT);
	LevelWindOff();

	// pin setup and disable reel at startup
	pinMode(MC1_ENABLE_PIN, OUTPUT);
	pinMode(MTR1_ENABLE_PIN, OUTPUT);
	ReelOff();

	// setup and disable shared logic enable
	pinMode(MC_ENABLE_PIN, OUTPUT);
	digitalWrite(MC_ENABLE_PIN, LOW);

	// setup and disable the brake enable pin
	pinMode(BRAKE_ENABLE_PIN, OUTPUT);
	digitalWrite(BRAKE_ENABLE_PIN, LOW);
}

void PowerControllerMCB::ReelOn()
{
	digitalWrite(MC_ENABLE_PIN, HIGH);
	delay(100);
	digitalWrite(MC1_ENABLE_PIN, HIGH);
	delay(100);
	digitalWrite(MTR1_ENABLE_PIN, HIGH);
	delay(100);
	digitalWrite(BRAKE_ENABLE_PIN, HIGH);
}

void PowerControllerMCB::ReelOff()
{
	digitalWrite(MC1_ENABLE_PIN, LOW);
	digitalWrite(MTR1_ENABLE_PIN, LOW);
	digitalWrite(BRAKE_ENABLE_PIN, LOW);

	// if the level wind controller is on, turn it off too
	if (!digitalRead(MC2_ENABLE_PIN)) {
		LevelWindOff();
	}

	digitalWrite(MC_ENABLE_PIN, LOW);
}

void PowerControllerMCB::LevelWindOn()
{
	// need the reel on for the level wind
	if (!digitalRead(MC1_ENABLE_PIN)) {
		ReelOn();
	}

	digitalWrite(MC2_ENABLE_PIN, LOW);
	delay(100);
	digitalWrite(MTR2_ENABLE_PIN, HIGH);
}

void PowerControllerMCB::LevelWindOff()
{
	digitalWrite(MC2_ENABLE_PIN, HIGH);
	digitalWrite(MTR2_ENABLE_PIN, LOW);
}
