/*
 *  StateManagerMCB.cpp
 *  File implementing the MCB state manager
 *  Author: Alex St. Clair
 *  February 2018
 */

#include "StateManagerMCB.h"

// --------------------------------------------------------
// Constructor
// --------------------------------------------------------

StateManagerMCB::StateManagerMCB()
    : action_queue(10)
	, monitor_queue(10)
	, dibDriver(&action_queue)
	, debugPort(&action_queue, &dibDriver)
	, powerController()
    , storageManager()
	, reel(1)
	, levelWind(1)
	, limitMonitor(&monitor_queue, &action_queue, &reel, &levelWind)
{
    last_pos_print = millis();
}

// --------------------------------------------------------
// Public interface functions
// --------------------------------------------------------

void StateManagerMCB::Startup()
{
	// Serial setup
	DEBUG_SERIAL.begin(115200);
	DIB_SERIAL.begin(115200);

	// Non-volatile storage setup
	storageManager.LoadFromEEPROM();
	storageManager.StartSD();

	// Set up limit monitor
	limitMonitor.InitializeSensors();
	limitMonitor.UpdateLimits();

	// Display startup info
	PrintBootInfo();

	// TTL/RS-232 transceiver setup
	pinMode(FORCEON_PIN, OUTPUT);
	pinMode(FORCEOFF_PIN, OUTPUT);
	digitalWrite(FORCEON_PIN, HIGH);
	digitalWrite(FORCEOFF_PIN, HIGH);
}

void StateManagerMCB::Loop()
{
	debugPort.RunDebugPort();
	dibDriver.RunDriver();
	PerformActions();
	RunState();
	limitMonitor.Monitor();
}

// --------------------------------------------------------
// State machine control
// --------------------------------------------------------

void StateManagerMCB::RunState(void)
{
	// check if there's a new state
	if (curr_state != last_state) {
		(this->*(state_array[last_state]))(true); // exit the old state
		num_loops = 0;
	}

	(this->*(state_array[curr_state]))(false); // call the current state
	last_state = curr_state;
	num_loops++;
}

bool StateManagerMCB::SetState(MCB_States_t new_state)
{
	if (new_state < 0 || new_state >= NUM_STATES) {
		return false;
	}

	curr_state = new_state;
	return true;
}

// --------------------------------------------------------
// Perform actions on queue
// --------------------------------------------------------

void StateManagerMCB::PerformActions(void)
{
	uint8_t action;
	while (!action_queue.IsEmpty()) {
		action = ACT_UNUSED;
		if (!action_queue.Pop(&action)) {
			return;
		}

		switch (action) {
		case ACT_SWITCH_READY:
			SetState(ST_READY);
			break;
		case ACT_SWITCH_NOMINAL:
			if (curr_state == ST_NOMINAL) {
				// todo: ack here?
			} else {
				SetState(ST_NOMINAL);
			}
			break;
		case ACT_DEPLOY_X:
			// only deploy if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_REEL_OUT);
			}
			// todo: error if already reeling?
			break;
		case ACT_RETRACT_X:
			// only retract if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_REEL_IN);
			}
			// todo: error if already reeling?
			break;
		case ACT_DOCK:
			// only dock if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_DOCK);
			}
			// todo: error if already reeling?
			break;
		case ACT_BRAKE_ON:
			reel.BrakeOn();
			break;
		case ACT_BRAKE_OFF:
			reel.BrakeOff();
			break;
		case ACT_SET_DV:
			EEPROM_UPDATE_FLOAT(storageManager, deploy_velocity, dibDriver.mcbParameters.deploy_velocity);
			break;
		case ACT_SET_RV:
			EEPROM_UPDATE_FLOAT(storageManager, retract_velocity, dibDriver.mcbParameters.retract_velocity);
			break;
		case ACT_SET_DA:
			EEPROM_UPDATE_FLOAT(storageManager, deploy_acceleration, dibDriver.mcbParameters.deploy_acceleration);
			break;
		case ACT_SET_RA:
			EEPROM_UPDATE_FLOAT(storageManager, retract_acceleration, dibDriver.mcbParameters.retract_acceleration);
			break;
		case ACT_ZERO_REEL:
			ReelControllerOn();
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				reel.SetPosition(0.0f);
			}
			ReelControllerOff();
			break;
		case ACT_LIMIT_EXCEEDED:
			if (curr_state != ST_READY) {
				storageManager.LogSD("Limit exceeded, setting state to ready", ERR_DATA);
				action_queue.Push(ACT_SWITCH_READY);
			}
			break;
		default:
			storageManager.LogSD("Unknown state manager action", ERR_DATA);
			break;
		}
	}
}

// --------------------------------------------------------
// Private helper methods
// --------------------------------------------------------

void StateManagerMCB::CheckReel(void)
{
	if (!reel.UpdateDriveStatus()) {
		storageManager.LogSD("Error updating reel drive status", ERR_DATA);
		return;
	}

	if (reel.drive_status.fault) {
		Serial.println("Motion fault (reel)");
		action_queue.Push(ACT_SWITCH_READY);
		dibDriver.dibComm.TX_ASCII(MCB_MOTION_FINISHED);
		LogFault();
	} else if (reel.drive_status.motion_complete) {
		Serial.println("Reel motion complete");
		action_queue.Push(ACT_SWITCH_READY);
		dibDriver.dibComm.TX_ASCII(MCB_MOTION_FINISHED);
	}
}


void StateManagerMCB::CheckLevelWind(void)
{
	if (!levelWind.UpdateDriveStatus()) {
		storageManager.LogSD("Error updating level wind drive status", ERR_DATA);
		return;
	}

	if (levelWind.drive_status.fault) {
		Serial.println("Motion fault (lw)");
		LogFault();
	} else if (reel.drive_status.motion_complete) {
		Serial.println("Level wind motion complete");
	}
}


void StateManagerMCB::LogFault(void)
{
	String fault_string = "";

	if (!reel.GenerateFaultInfo()) {
		storageManager.LogSD("Motion fault, unable to generate reel info", ERR_DATA);
	} else {
		String fault_string = "Motion fault rl:";
		fault_string += String(reel.drive_fault.status_lo, HEX);
		fault_string += ",";
		fault_string += String(reel.drive_fault.status_hi, HEX);
		fault_string += ",";
		fault_string += String(reel.drive_fault.motion_err, HEX);
		storageManager.LogSD(fault_string.c_str(), ERR_DATA);
	}

	if (!levelWind.GenerateFaultInfo()) {
		storageManager.LogSD("Motion fault, unable to generate level wind info", ERR_DATA);
	} else {
		String fault_string = "Motion fault lw:";
		fault_string += String(levelWind.drive_fault.status_lo, HEX);
		fault_string += ",";
		fault_string += String(levelWind.drive_fault.status_hi, HEX);
		fault_string += ",";
		fault_string += String(levelWind.drive_fault.motion_err, HEX);
		storageManager.LogSD(fault_string.c_str(), ERR_DATA);
	}
}


bool StateManagerMCB::ReelControllerOn(void)
{
	powerController.ReelOn();

	// wait for boot
	delay(500);

	// attempt communication
	if (!reel.Sync()) {
		powerController.ReelOff();
		action_queue.Push(ACT_SWITCH_READY);
		storageManager.LogSD("Unable to sync reel controller", ERR_DATA);
		return false;
	}

	// attempt to increase the baud rate
	// if (!reel.UpdateBaudRate(B115200)) {
	// 	powerController.ReelOff();
	// 	action_queue->Push(ACT_SWITCH_READY);
	// 	storageManager.LogSD("Error increasing reel controller baud", ERR_DATA);
	// 	return false;
	// }

	// start the drive control loops
	if (!reel.SendEndInit() || !reel.SetAxisOn()) {
		powerController.ReelOff();
		action_queue.Push(ACT_SWITCH_READY);
		storageManager.LogSD("Error starting reel control loops", ERR_DATA);
		return false;
	}

	// since the drive has (potentially) been turned off, reset the abs position to the stored one
	reel.SetToStoredPosition();

	reel_initialized = true;
	return true;
}


bool StateManagerMCB::LevelWindControllerOn(void)
{
	// reel must be started first
	if (!reel_initialized) return false;

	powerController.LevelWindOn();

	// wait for boot
	delay(500);

	// start the drive control loops
	if (!levelWind.SendEndInit() || !levelWind.SetAxisOn()) {
		powerController.LevelWindOff();
		action_queue.Push(ACT_SWITCH_READY);
		storageManager.LogSD("Error starting level wind control loops", ERR_DATA);
		return false;
	}

	levelwind_initialized = true;
	return true;
}


void StateManagerMCB::ReelControllerOff(void)
{
	powerController.ReelOff();
	reel_initialized = false;
}


void StateManagerMCB::LevelWindControllerOff(void)
{
	powerController.LevelWindOff();
	levelwind_initialized = false;
	homed = false;
	camming = false;
}

void StateManagerMCB::PrintBootInfo()
{
  DEBUG_SERIAL.print("MCB Boot ");
  DEBUG_SERIAL.println(storageManager.eeprom_data.boot_count);
  DEBUG_SERIAL.print("Hardware: Rev ");
  DEBUG_SERIAL.print(storageManager.eeprom_data.hardware_version[0]);
  DEBUG_SERIAL.print(" V");
  DEBUG_SERIAL.println(storageManager.eeprom_data.hardware_version[1]);
  DEBUG_SERIAL.print("Software: V");
  DEBUG_SERIAL.println(storageManager.eeprom_data.software_version);
}