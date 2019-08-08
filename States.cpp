/*
 *  States.cpp
 *  File implementing the MCB states as defined in MCB.h
 *  Author: Alex St. Clair
 *  August 2019
 */

#include "MCB.h"

// --------------------------------------------------------
// Substates
// --------------------------------------------------------

enum MCBSubstates_t : uint8_t{
	// State-generic
	STATE_ENTRY = ENTRY_SUBSTATE,
	STATE_EXIT = EXIT_SUBSTATE,

	// Ready
	READY_LOOP,

	// Nominal
	NOMINAL_LOOP,

	// Reel out
	REEL_OUT_START_MOTION,
	REEL_OUT_MONITOR,

	// Reel in
	REEL_IN_LW_ON,
	REEL_IN_START_MOTION,
	REEL_IN_HOME,
	REEL_IN_START_CAM,
	REEL_IN_MONITOR,

	// Dock
	DOCK_START_MOTION,
	DOCK_MONITOR,

	// Home LW
	HOME_START_MOTION,
	HOME_MONITOR,
};

// --------------------------------------------------------
// State methods
// --------------------------------------------------------

void MCB::Ready()
{
	switch (substate) {
	case STATE_ENTRY:
		Serial.println("Entering ready");
		substate = READY_LOOP;
		break;
	case READY_LOOP:
		// nothing to do
		break;
	case STATE_EXIT:
		Serial.println("Exiting ready");
		break;
	default:
		storageManager.LogSD("Unknown ready substate", ERR_DATA);
		substate = STATE_ENTRY;
		break;
	}
}


void MCB::Nominal()
{
	switch (substate) {
	case STATE_ENTRY:
		Serial.println("Entering nominal");
		LevelWindControllerOff();
		ReelControllerOff();
		monitor_queue.Push(MONITOR_LOW_POWER);
		// (?) dibDriver.dibComm.TX_Ack(MCB_GO_LOW_POWER);
		substate = NOMINAL_LOOP;
		break;
	case NOMINAL_LOOP:
		// nothing to do
		break;
	case STATE_EXIT:
		Serial.println("Exiting nominal");
		monitor_queue.Push(MONITOR_MOTORS_OFF);
		break;
	default:
		storageManager.LogSD("Unknown nominal substate", ERR_DATA);
		substate = STATE_ENTRY;
		break;
	}
}


void MCB::ReelOut()
{
	switch (substate) {
	case STATE_ENTRY:
		homed = false;
		camming = false;

		if (!ReelControllerOn()) {
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		monitor_queue.Push(MONITOR_REEL_ON);
		substate = REEL_OUT_START_MOTION;
		break;

	case REEL_OUT_START_MOTION:
		if (camming) {
			reel.CamStop();
			camming = false;
		}
		
		if (!reel.ReelOut(dibDriver.mcbParameters.deploy_length, dibDriver.mcbParameters.deploy_velocity, storageManager.eeprom_data.deploy_acceleration)) {
			Serial.println("Error reeling out");
			action_queue.Push(ACT_SWITCH_READY);
		}

		Serial.print("Reeling out: ");
		Serial.println(dibDriver.mcbParameters.deploy_length);

		substate = REEL_OUT_MONITOR;
		break;

	case REEL_OUT_MONITOR:
		// will switch mode once motion complete or fault detected
		CheckReel();

		if (millis() - last_pos_print > 5000) {
			Serial.println(reel.absolute_position / REEL_UNITS_PER_REV);
			last_pos_print = millis();
		}

		break;

	case STATE_EXIT:
		reel.StopProfile();
		// todo: store reel position
		ReelControllerOff();
		monitor_queue.Push(MONITOR_MOTORS_OFF);
		Serial.println("Exiting reel out");
		break;

	default:
		storageManager.LogSD("Unknown reel out substate", ERR_DATA);
		substate = STATE_ENTRY;
		// todo: better error handling
		break;
	}
}


void MCB::ReelIn()
{
	switch (substate) {
	case STATE_ENTRY:
		Serial.println("Entering reel in");
		
		if (!ReelControllerOn()) {
			Serial.println("Error powering reel on");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		substate = REEL_IN_LW_ON;
		break;

	case REEL_IN_LW_ON:
		if (!LevelWindControllerOn()) {
			Serial.println("Error powering LW on");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		monitor_queue.Push(MONITOR_BOTH_MOTORS_ON);
	
		substate = REEL_IN_START_MOTION;
		break;
		
	case REEL_IN_START_MOTION:
		if (!reel.ReelIn(dibDriver.mcbParameters.retract_length, dibDriver.mcbParameters.retract_velocity, storageManager.eeprom_data.retract_acceleration)) {
			Serial.println("Error reeling in");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		Serial.print("Reeling in: ");
		Serial.println(dibDriver.mcbParameters.retract_length);

		if (!homed) {
			if (!levelWind.Home()) {
				Serial.println("Error homing lw");
				action_queue.Push(ACT_SWITCH_READY);
			} else {
				substate = REEL_IN_HOME;
			}
		} else {
			substate = REEL_IN_START_CAM;
		}
		break;
		
	case REEL_IN_HOME:
		levelWind.UpdateDriveStatus();
		if (!(levelWind.drive_status.motion_complete || levelWind.drive_status.fault)) return;
		
		homed = true;
		
		if (!camming) {
			substate = REEL_IN_START_CAM;
		} else {
			substate = REEL_IN_MONITOR;
		}
		break;
		
	case REEL_IN_START_CAM:
		if (!reel.CamSetup() || !levelWind.StartCamming()) {
			reel.StopProfile();
			storageManager.LogSD("Error starting camming", ERR_DATA);
			action_queue.Push(ACT_SWITCH_READY);
			camming = false;
			return;
		}

		camming = true;
		substate = REEL_IN_MONITOR;
		break;
		
	case REEL_IN_MONITOR:
		// will exit once motion complete or fault
		CheckLevelWind();
		CheckReel();

		if (millis() - last_pos_print > 5000) {
			Serial.println(reel.absolute_position / REEL_UNITS_PER_REV);
			last_pos_print = millis();
		}

		break;
		
	case STATE_EXIT:
		reel.StopProfile();
		Serial.println("Exiting reel in");
		break;
	
	default:
		storageManager.LogSD("Unknown reel in substate", ERR_DATA);
		substate = STATE_ENTRY;
		// todo: better error handling
		break;
	}
}


void MCB::Dock()
{
	switch (substate) {
	case STATE_ENTRY:
		Serial.println("Entering dock");
		
		if (!reel_initialized || !levelwind_initialized || !homed) {
			storageManager.LogSD("Not ready for dock", ERR_DATA);
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		if (camming) {
			reel.CamStop();
			camming = false;
		}

		substate = DOCK_START_MOTION;
		break;

	case DOCK_START_MOTION:
		if (!reel.ReelIn(dibDriver.mcbParameters.dock_length, dibDriver.mcbParameters.dock_velocity, storageManager.eeprom_data.dock_acceleration)) {
			Serial.println("Error reeling in for dock");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		Serial.print("Docking: ");
		Serial.println(dibDriver.mcbParameters.dock_length);

		if (!levelWind.SetCenter()) {
			Serial.println("Error setting lw center for dock");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		substate = DOCK_MONITOR;
		break;

	case DOCK_MONITOR:
		CheckLevelWind();
		CheckReel();

		if (millis() - last_pos_print > 5000) {
			Serial.println(reel.absolute_position / REEL_UNITS_PER_REV);
			last_pos_print = millis();
		}

		break;

	case STATE_EXIT:
		Serial.println("Exiting dock");
		reel.StopProfile();
		levelWind.StopProfile();
		break;

	default:
		storageManager.LogSD("Unknown dock substate", ERR_DATA);
		substate = STATE_ENTRY;
		// todo: better error handling
		break;
	}
}


void MCB::HomeLW()
{
	switch (substate) {
	case STATE_ENTRY:
		Serial.println("Entering home lw");
		
		if (!ReelControllerOn() || !LevelWindControllerOn()) {
			Serial.println("Error powering controllers on");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		if (camming) {
			reel.CamStop();
			camming = false;
		}

		substate = DOCK_START_MOTION;
		break;

	case HOME_START_MOTION:
		if (!levelWind.Home()) {
			Serial.println("Error homing");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		substate = HOME_MONITOR;
		break;

	case HOME_MONITOR:
		CheckLevelWind();

		if (levelWind.drive_status.motion_complete) {
			action_queue.Push(ACT_SWITCH_READY);
		}

		break;

	case STATE_EXIT:
		Serial.println("Exiting home lw");
		reel.StopProfile();
		levelWind.StopProfile();
		break;

	default:
		storageManager.LogSD("Unknown home lw substate", ERR_DATA);
		substate = STATE_ENTRY;
		// todo: better error handling
		break;
	}
}