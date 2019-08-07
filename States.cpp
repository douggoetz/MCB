/*
 *  States.cpp
 *  File implementing the MCB states
 *  Author: Alex St. Clair
 *  August 2019
 */

#include "StateManagerMCB.h"

// --------------------------------------------------------
// State methods
// --------------------------------------------------------

void StateManagerMCB::Ready(bool exit)
{
	if (exit) {
		Serial.println("StateManager: exiting ready");
		return; // nothing to clean up
	}

	if (num_loops == 0) {
		Serial.println("StateManager: entering ready");
	}
}

void StateManagerMCB::Nominal(bool exit)
{
	if (exit) {
		Serial.println("StateManager: exiting nominal");
		monitor_queue.Push(MONITOR_MOTORS_OFF);
		return; // no cleanup
	}

	if (num_loops == 0) {
		Serial.println("StateManager: entering nominal");
		LevelWindControllerOff();
		ReelControllerOff();
		monitor_queue.Push(MONITOR_LOW_POWER);
		// TODO: should low power ack go here?
	}
}

void StateManagerMCB::ReelOut(bool exit)
{
	if (exit) {
		reel.StopProfile();
		levelWind.StopProfile();
		// todo: read/store current reel position
		Serial.println("StateManager: exiting reel out");
		ReelControllerOff();
		monitor_queue.Push(MONITOR_MOTORS_OFF);
		return;
	}

	if (num_loops == 0) {
		Serial.println("StateManager: entering reel out");
		homed = false;
		camming = false;

		if (!ReelControllerOn()) {
			Serial.println("Error turning reel on");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		monitor_queue.Push(MONITOR_REEL_ON);

	} else if (num_loops == 1) {
		if (camming) {
			reel.CamStop();
			camming = false;
		}

		if (!reel.ReelOut(dibDriver.mcbParameters.deploy_length, storageManager.eeprom_data.deploy_velocity, storageManager.eeprom_data.deploy_acceleration)) {
			Serial.println("StateManager: error reeling out");
			action_queue.Push(ACT_SWITCH_READY);
		}

		Serial.print("Reeling out: ");
		Serial.println(dibDriver.mcbParameters.deploy_length);

	} else {
		CheckReel();

		if (millis() - last_pos_print > 5000) {
			Serial.println(reel.absolute_position / REEL_UNITS_PER_REV);
			last_pos_print = millis();
		}
	}
}

void StateManagerMCB::ReelIn(bool exit)
{
	if (exit) {
		// stop all motion in case ongoing
		//reel.CamStop();
		reel.StopProfile();
		//levelWind.StopProfile();
		//camming = false;

		Serial.println("StateManager: exiting reel in");
		return;
	}

	if (num_loops == 0) {
		Serial.println("StateManager: entering reel in");
		
		if (!ReelControllerOn()) {
			Serial.println("Error powering reel on");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}
	
	} else if (num_loops == 1) {
		if (!LevelWindControllerOn()) {
			Serial.println("Error powering LW on");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		monitor_queue.Push(MONITOR_BOTH_MOTORS_ON);
	
	} else if (num_loops == 2) {
		if (!reel.ReelIn(dibDriver.mcbParameters.retract_length, storageManager.eeprom_data.retract_velocity, storageManager.eeprom_data.retract_acceleration)) {
			Serial.println("StateManager: error reeling in");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		Serial.print("Reeling in: ");
		Serial.println(dibDriver.mcbParameters.retract_length);

		if (!homed && !levelWind.Home()) {
			Serial.println("StateManager: error homing lw");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}
	
	} else if (!homed) {
		levelWind.UpdateDriveStatus();
		if (!levelWind.drive_status.motion_complete) return;
		
		homed = true;

	} else if (!camming) {
		if (!reel.CamSetup() || !levelWind.StartCamming()) {
			reel.StopProfile();
			Serial.println("Error starting camming");
			action_queue.Push(ACT_SWITCH_READY);
			camming = false;
			return;
		}

		camming = true;
	
	} else {
		CheckLevelWind();
		CheckReel();

		if (millis() - last_pos_print > 5000) {
			Serial.println(reel.absolute_position / REEL_UNITS_PER_REV);
			last_pos_print = millis();
		}
	}
}

void StateManagerMCB::Dock(bool exit)
{
	if (exit) {
		Serial.println("StateManager: exiting dock");
		reel.StopProfile();
		levelWind.StopProfile();
		// todo: if dock condition, reset reel postition to zero
		return; // nothing to clean up
	}

	if (num_loops == 0) {
		Serial.println("StateManager: entering dock");
		
		if (!reel_initialized || !levelwind_initialized || !homed) {
			Serial.println("StateManager: not ready for dock");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		if (camming) {
			reel.CamStop();
			camming = false;
		}

	} else if (num_loops == 1) {
		if (!reel.ReelIn(dibDriver.mcbParameters.dock_length, storageManager.eeprom_data.dock_velocity, storageManager.eeprom_data.dock_acceleration)) {
			Serial.println("StateManager: error reeling in for dock");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

		Serial.print("Docking: ");
		Serial.println(dibDriver.mcbParameters.dock_length);

		if (!levelWind.SetCenter()) {
			Serial.println("StateManager: error setting lw center for dock");
			action_queue.Push(ACT_SWITCH_READY);
			return;
		}

	} else {
		CheckLevelWind();
		CheckReel();

		if (millis() - last_pos_print > 5000) {
			Serial.println(reel.absolute_position / REEL_UNITS_PER_REV);
			last_pos_print = millis();
		}
	}
}