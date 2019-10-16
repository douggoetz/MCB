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

	// Reel in, no level wind
	IN_NO_LW_START_MOTION,
	IN_NO_LW_MONITOR,

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
		action_queue.Push(ACT_SWITCH_NOMINAL);
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
		dibDriver.dibComm.TX_Ack(MCB_GO_LOW_POWER, true);
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
		if (!limitMonitor.VerifyDeployVoltage()) {
			dibDriver.dibComm.TX_Error("Voltage too low to reel out!");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		if (!ReelControllerOn()) {
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		if (levelwind_initialized) {
			LevelWindControllerOff();
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
			dibDriver.dibComm.TX_Error("Error comanding reel out");
			action_queue.Push(ACT_SWITCH_NOMINAL);
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
		if (reel.StopProfile()) {
			delay(50); // wait a bit for motion to settle
			reel.UpdatePosition();
		}
		ReelControllerOff();
		monitor_queue.Push(MONITOR_MOTORS_OFF);
		Serial.println("Exiting reel out");
		break;

	default:
		storageManager.LogSD("Unknown reel out substate", ERR_DATA);
		action_queue.Push(ACT_SWITCH_NOMINAL);
		break;
	}
}


void MCB::ReelIn()
{
	// used for inter-loop timing
	static uint32_t timing_variable = 0;

	switch (substate) {
	case STATE_ENTRY:
		Serial.println("Entering reel in");

		if (!limitMonitor.VerifyDeployVoltage()) {
			dibDriver.dibComm.TX_Error("Voltage too low to reel in!");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		if (!ReelControllerOn()) {
			dibDriver.dibComm.TX_Error("Error powering reel on");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		substate = REEL_IN_LW_ON;
		break;

	case REEL_IN_LW_ON:
		if (!LevelWindControllerOn()) {
			dibDriver.dibComm.TX_Error("Error powering LW on");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		monitor_queue.Push(MONITOR_BOTH_MOTORS_ON);

		substate = REEL_IN_START_MOTION;
		break;

	case REEL_IN_START_MOTION:
		if (!reel.ReelIn(dibDriver.mcbParameters.retract_length, dibDriver.mcbParameters.retract_velocity, storageManager.eeprom_data.retract_acceleration)) {
			dibDriver.dibComm.TX_Error("Error commanding reel in");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		Serial.print("Reeling in: ");
		Serial.println(dibDriver.mcbParameters.retract_length);

		if (!homed) {
			if (!levelWind.Home()) {
				dibDriver.dibComm.TX_Error("Error commanding lw home");
				action_queue.Push(ACT_SWITCH_NOMINAL);
			} else {
				lw_docked = false;
				timing_variable = millis() + LW_HOME_MILLIS;
				substate = REEL_IN_HOME;
			}
		} else {
			substate = REEL_IN_START_CAM;
		}
		break;

	case REEL_IN_HOME:
		// make sure motion is complete and the limit switch has been hit before moving one
		// if thirty seconds passes, move on anyway
		if (levelWind.UpdateDriveStatus()) {
			if (!(levelWind.drive_status.lsp_event && levelWind.drive_status.motion_complete)) {
				if (millis() < timing_variable) return;
			}
		} else if (millis() < timing_variable) {
			return;
		}

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
			dibDriver.dibComm.TX_Error("Error starting camming");
			action_queue.Push(ACT_SWITCH_NOMINAL);
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
		if (reel.StopProfile()) {
			delay(50); // wait a bit for motion to settle if ongoing
			reel.UpdatePosition();
		} else {
			ReelControllerOff();
			LevelWindControllerOff();
			action_queue.Push(ACT_SWITCH_NOMINAL);
		}
		monitor_queue.Push(MONITOR_MOTORS_OFF);
		Serial.println("Exiting reel in");
		break;

	default:
		storageManager.LogSD("Unknown reel in substate", ERR_DATA);
		action_queue.Push(ACT_SWITCH_NOMINAL);
		break;
	}
}


void MCB::Dock()
{
	switch (substate) {
	case STATE_ENTRY:
		Serial.println("Entering dock");

		if (!limitMonitor.VerifyDeployVoltage()) {
			dibDriver.dibComm.TX_Error("Voltage too low to dock!");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		if (!reel_initialized || !levelwind_initialized || (!homed && !lw_docked)) {
			dibDriver.dibComm.TX_Error("Not ready for dock");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		if (camming) {
			reel.CamStop();
			camming = false;
		}

		monitor_queue.Push(MONITOR_BOTH_MOTORS_ON);

		substate = DOCK_START_MOTION;
		break;

	case DOCK_START_MOTION:
		if (!reel.ReelIn(dibDriver.mcbParameters.dock_length, dibDriver.mcbParameters.dock_velocity, storageManager.eeprom_data.dock_acceleration)) {
			dibDriver.dibComm.TX_Error("Error commanding reel in for dock");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		Serial.print("Docking: ");
		Serial.println(dibDriver.mcbParameters.dock_length);

		if (!lw_docked && !levelWind.SetCenter()) {
			dibDriver.dibComm.TX_Error("Error setting lw center for dock");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		substate = DOCK_MONITOR;
		break;

	case DOCK_MONITOR:
		if (CheckLevelWind()) lw_docked = true;
		CheckReel();

		if (millis() - last_pos_print > 5000) {
			Serial.println(reel.absolute_position / REEL_UNITS_PER_REV);
			last_pos_print = millis();
		}

		break;

	case STATE_EXIT:
		if (reel.StopProfile() && levelWind.StopProfile()) {
			delay(50); // wait a bit for motion to settle if ongoing
			reel.UpdatePosition();
		} else {
			ReelControllerOff();
			LevelWindControllerOff();
			action_queue.Push(ACT_SWITCH_NOMINAL);
		}
		homed = false;
		monitor_queue.Push(MONITOR_MOTORS_OFF);
		break;

	default:
		storageManager.LogSD("Unknown dock substate", ERR_DATA);
		action_queue.Push(ACT_SWITCH_NOMINAL);
		break;
	}
}


void MCB::InNoLW()
{
	switch (substate) {
	case STATE_ENTRY:
		if (!limitMonitor.VerifyDeployVoltage()) {
			dibDriver.dibComm.TX_Error("Voltage too low to reel in (no LW)!");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		if (!ReelControllerOn()) {
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		if (levelwind_initialized) {
			LevelWindControllerOff();
		}

		monitor_queue.Push(MONITOR_REEL_ON);
		substate = IN_NO_LW_START_MOTION;
		break;

	case IN_NO_LW_START_MOTION:
		if (camming) {
			reel.CamStop();
			camming = false;
		}

		if (!reel.ReelIn(dibDriver.mcbParameters.retract_length, dibDriver.mcbParameters.retract_velocity, storageManager.eeprom_data.retract_acceleration)) {
			dibDriver.dibComm.TX_Error("Error comanding reel in (no LW)");
			action_queue.Push(ACT_SWITCH_NOMINAL);
		}

		Serial.print("Reeling in (no LW): ");
		Serial.println(dibDriver.mcbParameters.retract_length);

		substate = IN_NO_LW_MONITOR;
		break;

	case IN_NO_LW_MONITOR:
		// will switch mode once motion complete or fault detected
		CheckReel();

		if (millis() - last_pos_print > 5000) {
			Serial.println(reel.absolute_position / REEL_UNITS_PER_REV);
			last_pos_print = millis();
		}

		break;

	case STATE_EXIT:
		if (reel.StopProfile()) {
			delay(50); // wait a bit for motion to settle
			reel.UpdatePosition();
		}
		ReelControllerOff();
		monitor_queue.Push(MONITOR_MOTORS_OFF);
		Serial.println("Exiting reel in (no LW)");
		break;

	default:
		storageManager.LogSD("Unknown reel in (no LW) substate", ERR_DATA);
		action_queue.Push(ACT_SWITCH_NOMINAL);
		break;
	}
}


void MCB::HomeLW()
{
	static uint32_t timing_variable = 0;

	switch (substate) {
	case STATE_ENTRY:
		Serial.println("Entering home lw");

		if (!limitMonitor.VerifyDeployVoltage()) {
			dibDriver.dibComm.TX_Error("Voltage too low to home!");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		if (!ReelControllerOn() || !LevelWindControllerOn()) {
			dibDriver.dibComm.TX_Error("Error powering controllers on for LW home");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		if (camming) {
			reel.CamStop();
			camming = false;
		}

		substate = HOME_START_MOTION;
		break;

	case HOME_START_MOTION:
		if (!levelWind.Home()) {
			dibDriver.dibComm.TX_Error("Error homing LW");
			action_queue.Push(ACT_SWITCH_NOMINAL);
			return;
		}

		timing_variable = millis() + LW_HOME_MILLIS;
		lw_docked = false;
		substate = HOME_MONITOR;
		break;

	case HOME_MONITOR:
		CheckLevelWind();

		if (millis() > timing_variable) {
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
		action_queue.Push(ACT_SWITCH_NOMINAL);
		break;
	}
}