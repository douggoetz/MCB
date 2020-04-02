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
	, powerController()
    , storageManager()
	, configManager()
	, dibDriver(&action_queue, &monitor_queue)
	, reel(1)
	, levelWind(1)
	, limitMonitor(&monitor_queue, &action_queue, &reel, &levelWind, &dibDriver, &configManager)
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
	if (!configManager.Initialize()) dibDriver.dibComm.TX_Error("MCB error initializing EEPROM! Reconfigured");
	if (!storageManager.StartSD(configManager.boot_count.Read())) dibDriver.dibComm.TX_Error("MCB error starting SD card!");

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
	dibDriver.RunDebugDriver();
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
        dibDriver.dibComm.TX_Error("MCB reset caused by watchdog");
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
				substate = ENTRY_SUBSTATE; // restart nominal, not a problem, will send ACK
			} else {
				SetState(ST_NOMINAL);
			}
			break;
		case ACT_DEPLOY_X:
			// only deploy if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_REEL_OUT);
				dibDriver.dibComm.TX_Ack(MCB_REEL_OUT, true);
			} else {
				dibDriver.dibComm.TX_Error("Deploy denied, reel ops ongoing");
			}
			break;
		case ACT_RETRACT_X:
			// only retract if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_REEL_IN);
				dibDriver.dibComm.TX_Ack(MCB_REEL_IN, true);
			} else {
				dibDriver.dibComm.TX_Error("Retract denied, reel ops ongoing");
			}
			break;
		case ACT_DOCK:
			// only dock if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_DOCK);
				dibDriver.dibComm.TX_Ack(MCB_DOCK, true);
			} else {
				dibDriver.dibComm.TX_Error("Dock denied, reel ops ongoing");
			}
			break;
		case ACT_IN_NO_LW:
			// only dock if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_IN_NO_LW);
				dibDriver.dibComm.TX_Ack(MCB_IN_NO_LW, true);
			} else {
				dibDriver.dibComm.TX_Error("Reel in no LW denied, reel ops ongoing");
			}
			break;
		case ACT_HOME_LW:
			// only home if not currently performing reel operation
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				SetState(ST_HOME_LW);
			} else {
				dibDriver.dibComm.TX_Error("Home denied, reel ops ongoing");
			}
			break;
		case ACT_BRAKE_ON:
			reel.BrakeOn();
			break;
		case ACT_BRAKE_OFF:
			if (reel_initialized) {
				reel.BrakeOff();
			} else {
				dibDriver.dibComm.TX_Error("Reel not initialized for brake");
			}
			break;
		case ACT_CONTROLLERS_ON:
			if (curr_state == ST_READY) {
				if (!ReelControllerOn() || !LevelWindControllerOn()) {
					dibDriver.dibComm.TX_Error("Error powering controllers");
				}
			} else {
				dibDriver.dibComm.TX_Error("Wrong state for powering controllers");
			}
			break;
		case ACT_CONTROLLERS_OFF:
			ReelControllerOff();
			LevelWindControllerOff();
			break;
		case ACT_SET_DEPLOY_V:
			configManager.deploy_velocity.Write(dibDriver.mcbParameters.deploy_velocity);
			break;
		case ACT_SET_RETRACT_V:
			configManager.retract_velocity.Write(dibDriver.mcbParameters.retract_velocity);
			break;
		case ACT_SET_DOCK_V:
			configManager.dock_velocity.Write(dibDriver.mcbParameters.dock_velocity);
			break;
		case ACT_SET_DEPLOY_A:
			configManager.deploy_acceleration.Write(dibDriver.mcbParameters.deploy_acceleration);
			dibDriver.dibComm.TX_Ack(MCB_OUT_ACC,true);
			break;
		case ACT_SET_RETRACT_A:
			configManager.retract_acceleration.Write(dibDriver.mcbParameters.retract_acceleration);
			dibDriver.dibComm.TX_Ack(MCB_IN_ACC,true);
			break;
		case ACT_SET_DOCK_A:
			configManager.dock_acceleration.Write(dibDriver.mcbParameters.dock_acceleration);
			dibDriver.dibComm.TX_Ack(MCB_DOCK_ACC,true);
			break;
		case ACT_ZERO_REEL:
			Serial.println("Zeroing reel");
			if (curr_state == ST_NOMINAL || curr_state == ST_READY) {
				ReelControllerOn();
				if (reel.SetPosition(0.0f)) {
					dibDriver.dibComm.TX_Ack(MCB_ZERO_REEL, true);
				} else {
					dibDriver.dibComm.TX_Ack(MCB_ZERO_REEL, false);
				}
				ReelControllerOff();
			}
			break;
		case ACT_LIMIT_EXCEEDED:
			if (curr_state != ST_READY && curr_state != ST_NOMINAL) {
				dibDriver.dibComm.TX_Error(limitMonitor.limit_error);
				action_queue.Push(ACT_SWITCH_READY);
			}
			break;
		case ACT_FULL_RETRACT:
			// in case motion is ongoing, kill the controllers
			Serial.println("Full retract command received");
			LevelWindControllerOff();
			ReelControllerOff();

			// trust the position in memory, and reel in 99.5% of it at the default speed if it shows we're deployed
			if (reel.absolute_position < 0) {
				dibDriver.mcbParameters.retract_length = 0.995 * abs(reel.absolute_position) / REEL_UNITS_PER_REV;
				dibDriver.mcbParameters.retract_velocity = DEFAULT_FULL_SPEED;
				SetState(ST_REEL_IN);
			} else {
				Serial.println("Full retract unnecessary, already reeled in");

				// send the motion complete twice
				dibDriver.dibComm.TX_ASCII(MCB_MOTION_FINISHED);
				delay(100);
				dibDriver.dibComm.TX_ASCII(MCB_MOTION_FINISHED);
			}

			dibDriver.dibComm.TX_Ack(MCB_FULL_RETRACT,true);

			break;
		case ACT_TEMP_LIMITS:
			configManager.mtr1_temp_lim.Write(Limit_Config_t(dibDriver.mcbParameters.temp_limits[0],dibDriver.mcbParameters.temp_limits[1]));
			configManager.mtr2_temp_lim.Write(Limit_Config_t(dibDriver.mcbParameters.temp_limits[2],dibDriver.mcbParameters.temp_limits[3]));
			configManager.mc1_temp_lim.Write(Limit_Config_t(dibDriver.mcbParameters.temp_limits[4],dibDriver.mcbParameters.temp_limits[5]));
			limitMonitor.temp_sensors[MTR1_THERM].limit_hi = dibDriver.mcbParameters.temp_limits[0];
			limitMonitor.temp_sensors[MTR1_THERM].limit_lo = dibDriver.mcbParameters.temp_limits[1];
			limitMonitor.temp_sensors[MTR2_THERM].limit_hi = dibDriver.mcbParameters.temp_limits[2];
			limitMonitor.temp_sensors[MTR2_THERM].limit_lo = dibDriver.mcbParameters.temp_limits[3];
			limitMonitor.temp_sensors[MC1_THERM].limit_hi = dibDriver.mcbParameters.temp_limits[4];
			limitMonitor.temp_sensors[MC1_THERM].limit_lo = dibDriver.mcbParameters.temp_limits[5];
			dibDriver.dibComm.TX_Ack(MCB_TEMP_LIMITS,true);
			break;
		case ACT_TORQUE_LIMITS:
			configManager.reel_torque_lim.Write(Limit_Config_t(dibDriver.mcbParameters.torque_limits[0],dibDriver.mcbParameters.torque_limits[1]));
			limitMonitor.motor_torques[0].limit_hi = dibDriver.mcbParameters.torque_limits[0];
			limitMonitor.motor_torques[0].limit_lo = dibDriver.mcbParameters.torque_limits[1];
			dibDriver.dibComm.TX_Ack(MCB_TORQUE_LIMITS,true);
			break;
		case ACT_CURR_LIMITS:
			configManager.imon_mtr1_lim.Write(Limit_Config_t(dibDriver.mcbParameters.curr_limits[0],dibDriver.mcbParameters.curr_limits[1]));
			limitMonitor.imon_channels[IMON_MTR1].limit_hi = dibDriver.mcbParameters.curr_limits[0];
			limitMonitor.imon_channels[IMON_MTR1].limit_lo = dibDriver.mcbParameters.curr_limits[1];
			dibDriver.dibComm.TX_Ack(MCB_CURR_LIMITS,true);
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
		action_queue.Push(ACT_SWITCH_NOMINAL);
		LogFault();
	} else if (reel.drive_status.motion_complete) {
#ifdef INST_FLOATS
		levelWind.StopProfile();
#endif
		Serial.println("Reel motion complete");
		action_queue.Push(ACT_SWITCH_READY);
		dibDriver.dibComm.TX_ASCII(MCB_MOTION_FINISHED);
		delay(100);
		dibDriver.dibComm.TX_ASCII(MCB_MOTION_FINISHED); // tx twice in case DIB misses the first
	}
}

// returns true if and only if motion complete
bool MCB::CheckLevelWind(void)
{
	if (!levelWind.UpdateDriveStatus()) {
		storageManager.LogSD("Error updating level wind drive status", ERR_DATA);
		return false;
	}

	if (levelWind.drive_status.fault) {
		action_queue.Push(ACT_SWITCH_NOMINAL);
		Serial.println("Motion fault (lw)");
		LogFault();
		return false;
	} else if (levelWind.drive_status.motion_complete) {
		return true;
	}
	return false;
}

#ifdef INST_FLOATS
bool MCB::CheckLevelWindCam(void)
{
	if (!levelWind.UpdateDriveStatus()) {
		storageManager.LogSD("Error updating level wind drive status", ERR_DATA);
		return false;
	}

	if (levelWind.drive_status.lsp_event) homed = true;

	if (levelWind.drive_status.fault) {
		action_queue.Push(ACT_SWITCH_NOMINAL);
		Serial.println("Motion fault (lw)");
		LogFault();
	} else if (!lw_direction_out && levelWind.drive_status.motion_complete) {
		if (!levelWind.WindOut()) {
			action_queue.Push(ACT_SWITCH_NOMINAL);
			Serial.println("Unable to command next lw segment");
		}
		lw_direction_out ^= true; // flip direction tracker
	} else if (lw_direction_out && levelWind.drive_status.lsp_event && levelWind.drive_status.motion_complete) {
		if (!levelWind.WindIn()) {
			action_queue.Push(ACT_SWITCH_NOMINAL);
			Serial.println("Unable to command next lw segment");
		}
		lw_direction_out ^= true; // flip direction tracker
	}

	return false; // this should never return true for FLOATS
}
#endif


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

	dibDriver.dibComm.TX_Motion_Fault(reel.drive_fault.status_lo, reel.drive_fault.status_hi,
									  reel.drive_fault.detailed_err, reel.drive_fault.motion_err,
									  levelWind.drive_fault.status_lo, levelWind.drive_fault.status_hi,
									  levelWind.drive_fault.detailed_err, levelWind.drive_fault.motion_err);

	delay(100);

	// send twice in case the DIB misses the first
	dibDriver.dibComm.TX_Motion_Fault(reel.drive_fault.status_lo, reel.drive_fault.status_hi,
									  reel.drive_fault.detailed_err, reel.drive_fault.motion_err,
									  levelWind.drive_fault.status_lo, levelWind.drive_fault.status_hi,
									  levelWind.drive_fault.detailed_err, levelWind.drive_fault.motion_err);
}


bool MCB::ReelControllerOn(void)
{
	if (reel_initialized) {
		if (!reel.SendEndInit() || !reel.SetAxisOn()) {
			powerController.ReelOff();
			action_queue.Push(ACT_SWITCH_READY);
			storageManager.LogSD("Error starting reel control loops", ERR_DATA);
			dibDriver.dibComm.TX_Error("Error starting reel control loops");
			return false;
		}
		return true;
	}

	powerController.ReelOn();

	// wait for boot
	delay(500);

	// attempt communication
	if (!reel.Sync()) {
		powerController.ReelOff();
		action_queue.Push(ACT_SWITCH_READY);
		storageManager.LogSD("Unable to sync reel controller", ERR_DATA);
		dibDriver.dibComm.TX_Error("Unable to sync reel controller");
		return false;
	}

	// attempt to increase the baud rate
	// if (!reel.UpdateBaudRate(B115200)) {
	// 	powerController.ReelOff();
	// 	action_queue.Push(ACT_SWITCH_READY);
	// 	storageManager.LogSD("Error increasing reel controller baud", ERR_DATA);
	// 	dibDriver.dibComm.TX_Error("Error increasing reel controller baud");
	// 	return false;
	// }

	// start the drive control loops
	if (!reel.SendEndInit() || !reel.SetAxisOn()) {
		powerController.ReelOff();
		action_queue.Push(ACT_SWITCH_READY);
		storageManager.LogSD("Error starting reel control loops", ERR_DATA);
		dibDriver.dibComm.TX_Error("Error starting reel control loops");
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
		dibDriver.dibComm.TX_Error("Error starting level wind control loops");
		return false;
	}

	// restart the reel drive control loops
	if (!reel.SendEndInit() || !reel.SetAxisOn()) {
		powerController.LevelWindOff();
		powerController.ReelOff();
		action_queue.Push(ACT_SWITCH_READY);
		storageManager.LogSD("Error starting reel control loops", ERR_DATA);
		dibDriver.dibComm.TX_Error("Error starting reel control loops in level wind on");
		return false;
	}

	levelwind_initialized = true;
	return true;
}


void MCB::ReelControllerOff(void)
{
	powerController.ReelOff();
	reel_initialized = false;
	camming = false;
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
  DEBUG_SERIAL.println(configManager.boot_count.Read());
}