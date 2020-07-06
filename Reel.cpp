/*
 *  Reel.h
 *  Implementation of a class to control the reel
 *  Author: Alex St. Clair
 *  January 2018
 *
 *  This file defines an Arduino library (C++ class) that controls
 *  the reel. It inherits from the Technosoft class, which implements
 *  communication with the Technosoft motor controllers.
 */

#include <Reel.h>

Reel::Reel(uint8_t expeditor_axis) : Technosoft(REEL_AXIS, expeditor_axis) {
	// keep emergency stop off to start
	pinMode(MC1_IN4_PIN, OUTPUT); // limit switch negative (emergency stop)
	digitalWrite(MC1_IN4_PIN, HIGH);
	absolute_position = 0;
	speed = 0.0f;
}

bool Reel::SetPosition(float new_pos) {
	int32_t cmd_pos = (int32_t) (new_pos * REEL_UNITS_PER_REV);
	if (SetAbsolutePosition(cmd_pos)) {
		absolute_position = cmd_pos;
		UpdatePosition(); // will log to file
		return true;
	}
	// TODO: error checking
	return false;
}

bool Reel::SetPosition(int32_t new_pos) {
	if (SetAbsolutePosition(new_pos)) {
		absolute_position = new_pos;
		UpdatePosition(); // will log to file
		return true;
	}
	// TODO: error checking
	return false;
}

bool Reel::UpdatePosition() {
	int32_t new_pos = ReadAbsolutePosition();
	if (new_pos != (int32_t) 0xFFFFFFFF) {
		absolute_position = new_pos;
		storageManager.WriteSD_int32(POS_LOG_FILE, new_pos, false, 0);
		return true;
	}
	return false;
}

bool Reel::UpdateSpeed() {
	speed = ReadActualSpeed();
	return 0.0f == speed;
}

void Reel::SetToStoredPosition() {
	if (!storageManager.FileExists(POS_LOG_FILE)) {
		SetPosition((int32_t) absolute_position);
		return;
	}

	int32_t read_pos = 0;
	if (storageManager.ReadSD_int32(POS_LOG_FILE, &read_pos, 0)) {
		SetPosition(read_pos);
	} else {
		SetPosition((int32_t) absolute_position);
	}
}

bool Reel::StopProfile() {
	return CallFunction(STOP_PROFILE_R);
}

bool Reel::ReelIn(float num_revolutions, float speed, float acc) {
	uint32_t num_units = 0;
	uint32_t fixed_speed = 0;
	uint32_t fixed_acc = 0;

	if (num_revolutions > MAX_REVOLUTIONS || num_revolutions <= 0.0) { return false; }
	if (speed > MAX_SPEED || speed <= 0.0) { return false; }
	if (acc > MAX_ACC || acc <= 0.0) { return false; }

	// implicit cast to uint32 for serialization
	num_units = num_revolutions * REEL_UNITS_PER_REV;

	// Technosoft unit conversions
	speed = speed * SPEED_CONVERSION;
	acc = acc * ACC_CONVERSION;

	// convert floats to fixed points
	fixed_speed = Float_To_Fixed(speed);
	fixed_acc = Float_To_Fixed(acc);

	if (!SetCommandPosition(num_units)) { return false; }
	if (!SetSlewRate(fixed_speed)) { return false; }
	if (!SetAcceleration(fixed_acc)) { return false; }

	return CallFunction(REEL_VARIABLE_R);
}

bool Reel::ReelOut(float num_revolutions, float speed, float acc) {
	uint32_t num_units = 0;
	uint32_t fixed_speed = 0;
	uint32_t fixed_acc = 0;

	if (num_revolutions > MAX_REVOLUTIONS || num_revolutions <= 0.0) { return false; }
	if (speed > MAX_SPEED || speed <= 0.0) { return false; }
	if (acc > MAX_ACC || acc <= 0.0) { return false; }

	// cast as int32 first to get sign before implicit cast to uint32 for serialization
	num_units = (int32_t) (num_revolutions * REEL_UNITS_PER_REV * -1);

	// Technosoft unit conversions
	speed = speed * SPEED_CONVERSION;
	acc = acc * ACC_CONVERSION;

	// convert floats to fixed points
	fixed_speed = Float_To_Fixed(speed);
	fixed_acc = Float_To_Fixed(acc);

	if (!SetCommandPosition(num_units)) { return false; }
	if (!SetSlewRate(fixed_speed)) { return false; }
	if (!SetAcceleration(fixed_acc)) { return false; }

	return CallFunction(REEL_VARIABLE_R);
}

bool Reel::CamSetup() {
	return CallFunction(CAM_SETUP_R);
}

bool Reel::CamStop() {
	return CallFunction(CAM_STOP_R);
}

bool Reel::BrakeOn() {
	return CallFunction(BRAKE_ON_R);
}

bool Reel::BrakeOff() {
	return CallFunction(BRAKE_OFF_R);
}

void Reel::EmergencyStop() {
	digitalWrite(MC1_IN4_PIN, LOW);
}

void Reel::RemoveEmergencyStop() {
	digitalWrite(MC1_IN4_PIN, HIGH);
}