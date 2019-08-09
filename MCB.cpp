/*
 *  MCB.cpp
 *  File implementing the MCB state manager
 *  Author: Alex St. Clair
 *  February 2018
 */

#include "MCB.h"

// --------------------------------------------------------
// Constructor
// --------------------------------------------------------

MCB::MCB()
    : action_queue(10)
	, monitor_queue(10)
	, dibDriver(&action_queue, &monitor_queue)
	, debugPort(&action_queue, &dibDriver)
	, powerController()
    , storageManager()
	, reel(1)
	, levelWind(1)
	, limitMonitor(&monitor_queue, &action_queue, &reel, &levelWind, &dibDriver)
{
    last_pos_print = millis();
}

// --------------------------------------------------------
// Public interface functions
// --------------------------------------------------------

void MCB::Startup()
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

	InitializeWatchdog();
}

// note that the loop timing is controlled in MCB_Main.ino
void MCB::Loop()
{
	debugPort.RunDebugPort();
	dibDriver.RunDriver();
	PerformActions();
	RunState();
	limitMonitor.Monitor();
	KickWatchdog();
}

// --------------------------------------------------------
// State machine control
// --------------------------------------------------------

void MCB::RunState(void)
{
	// check if there's a new state
	if (curr_state != last_state) {
		substate = EXIT_SUBSTATE;
		(this->*(state_array[last_state]))(); // exit the old state
		substate = ENTRY_SUBSTATE;
	}

	(this->*(state_array[curr_state]))(); // call the current state
	last_state = curr_state;
}

bool MCB::SetState(MCB_States_t new_state)
{
	if (new_state < 0 || new_state >= NUM_STATES) {
		return false;
	}

	curr_state = new_state;
	return true;
}

// --------------------------------------------------------
// Watchdog
// --------------------------------------------------------

// initialize the watchdog using the 1kHz LPO clock source to achieve a 10s WDOG
void MCB::InitializeWatchdog()
{
    if ((RCM_SRS0 & RCM_SRS0_WDOG) != 0) {
        storageManager.LogSD("Reset caused by watchdog", ERR_DATA);
    }

    noInterrupts(); // disable interrupts

    // unlock
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ1; // unlock access to WDOG registers
    WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
    delayMicroseconds(1);

    WDOG_PRESC = 0; // no prescaling of clock

    WDOG_TOVALH = 0x0000; // upper bits set to 0
    WDOG_TOVALL = 0x2710; // 10000 counter at 1 kHz => 10s WDOG period

    // in one write, enable the watchdog using the 1kHz LPO clock source
    WDOG_STCTRLH = 0x01D1;

    interrupts(); // enable interrupts
}

void MCB::KickWatchdog()
{
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
}

// --------------------------------------------------------
// Perform actions on queue
// --------------------------------------------------------

void MCB::PerformActions(void)
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
			dibDriver.dibComm.TX_Error("Deploy denied, reel ops ongoing");
			break;
		case ACT_RETRACT_X:
			// only retract if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_REEL_IN);
			}
			dibDriver.dibComm.TX_Error("Retract denied, reel ops ongoing");
			break;
		case ACT_DOCK:
			// only dock if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_DOCK);
			}
			dibDriver.dibComm.TX_Error("Dock denied, reel ops ongoing");
			break;
		case ACT_HOME_LW:
			// only home if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_HOME_LW);
			}
			dibDriver.dibComm.TX_Error("Home denied, reel ops ongoing");
			break;
		case ACT_BRAKE_ON:
			reel.BrakeOn();
			break;
		case ACT_BRAKE_OFF:
			reel.BrakeOff();
			break;
		case ACT_SET_DEPLOY_V:
			EEPROM_UPDATE_FLOAT(storageManager, deploy_velocity, dibDriver.mcbParameters.deploy_velocity);
			break;
		case ACT_SET_RETRACT_V:
			EEPROM_UPDATE_FLOAT(storageManager, retract_velocity, dibDriver.mcbParameters.retract_velocity);
			break;
		case ACT_SET_DOCK_V:
			EEPROM_UPDATE_FLOAT(storageManager, dock_velocity, dibDriver.mcbParameters.dock_velocity);
			break;
		case ACT_SET_DEPLOY_A:
			EEPROM_UPDATE_FLOAT(storageManager, deploy_acceleration, dibDriver.mcbParameters.deploy_acceleration);
			break;
		case ACT_SET_RETRACT_A:
			EEPROM_UPDATE_FLOAT(storageManager, retract_acceleration, dibDriver.mcbParameters.retract_acceleration);
			break;
		case ACT_SET_DOCK_A:
			EEPROM_UPDATE_FLOAT(storageManager, dock_acceleration, dibDriver.mcbParameters.dock_acceleration);
			break;
		case ACT_ZERO_REEL:
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				ReelControllerOn();
				reel.SetPosition(0.0f);
				ReelControllerOff();
			}
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

void MCB::CheckReel(void)
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


void MCB::CheckLevelWind(void)
{
	if (!levelWind.UpdateDriveStatus()) {
		storageManager.LogSD("Error updating level wind drive status", ERR_DATA);
		return;
	}

	if (levelWind.drive_status.fault) {
		action_queue.Push(ACT_SWITCH_READY);
		Serial.println("Motion fault (lw)");
		LogFault();
	} else if (levelWind.drive_status.motion_complete) {
		Serial.println("Level wind motion complete");
	}
}


void MCB::LogFault(void)
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


bool MCB::ReelControllerOn(void)
{
	if (reel_initialized) return true;

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


bool MCB::LevelWindControllerOn(void)
{
	if (levelwind_initialized) return true;

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


void MCB::ReelControllerOff(void)
{
	powerController.ReelOff();
	reel_initialized = false;
}


void MCB::LevelWindControllerOff(void)
{
	powerController.LevelWindOff();
	levelwind_initialized = false;
	homed = false;
	camming = false;
}

void MCB::PrintBootInfo()
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