/*
 *  LevelWind.h
 *  Implementation of a class to control the level wind
 *  Author: Alex St. Clair
 *  January 2018
 *
 *  This file defines an Arduino library (C++ class) that controls
 *  the level wind. It inherits from the Technosoft class, which
 *  implements communication with the Technosoft motor controllers.
 */

#include <LevelWind.h>

LevelWind::LevelWind(uint8_t expeditor_axis) : Technosoft(LEVEL_WIND_AXIS, expeditor_axis) { }

bool LevelWind::StopProfile() {
	return CallFunction(STOP_PROFILE_LW);
}

bool LevelWind::SetCenter() {
	return CallFunction(SET_CENTER_LW);
}

bool LevelWind::Home() {
	return CallFunction(HOME_LW);
}

bool LevelWind::UpdatePosition() {
	int32_t new_pos = ReadAbsolutePosition();
	if (new_pos != (int32_t) 0xFFFFFFFF) {
		absolute_position = (float) new_pos / LW_STEPS_PER_MM;
		return true;
	}
	return false;
}

bool LevelWind::WindOut(float reel_speed) {
	if (!SetSlewRate(reel_speed*LW_SPEED_CONV)) { return false; }
	delay(50);
	return CallFunction(WIND_OUT_LW);
}

bool LevelWind::WindIn(float reel_speed) {
	if (!SetSlewRate(reel_speed*LW_SPEED_CONV)) { return false; }
	delay(50);
	return CallFunction(WIND_IN_LW);
}
