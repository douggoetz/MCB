/*
 *  InternalSerialDriverMCB.cpp
 *  File implementing the serial driver for communicating with the
 *  main payload computer (DIB) from the MCB.
 *  Author: Alex St. Clair
 *  February 2018
 */

#include "InternalSerialDriverMCB.h"

InternalSerialDriverMCB::InternalSerialDriverMCB(Queue * state_q, Queue * monitor_q)
    : dibComm(&DIB_SERIAL)
    , storageManager()
{
    state_queue = state_q;
	monitor_queue = monitor_q;
}

void InternalSerialDriverMCB::RunDriver(void)
{
	SerialMessage_t rx_msg = dibComm.RX();

	while (rx_msg != NO_MESSAGE) {
		if (rx_msg == ASCII_MESSAGE) {
			HandleASCII();
		} else {
			storageManager.LogSD("Unimplemented message type from DIB", ERR_DATA);
		}

		rx_msg = dibComm.RX();
	}
}

void InternalSerialDriverMCB::RunDebugDriver(void)
{
	SerialMessage_t rx_msg = NO_MESSAGE;
	char peek_char = '\0';

	// switch to USB serial for those messages
	dibComm.UpdatePort(&Serial);

	peek_char = Serial.peek();
	if ('m' == peek_char) {
		Serial.read(); // clear the 'm'
		PrintDebugMenu();
	} else {
		rx_msg = dibComm.RX();
		while (rx_msg != NO_MESSAGE) {
			if (rx_msg == ASCII_MESSAGE) {
				HandleASCII();
			}

			rx_msg = dibComm.RX();
		}
	}

	// switch back to the DIB
	dibComm.UpdatePort(&DIB_SERIAL);
}

void InternalSerialDriverMCB::HandleASCII(void)
{
	switch (dibComm.ascii_rx.msg_id) {
	// messages with no parameters ------------------------
	case MCB_CANCEL_MOTION:
		state_queue->Push(ACT_SWITCH_READY);
		dibComm.TX_Ack(MCB_CANCEL_MOTION, true);
		break;
	case MCB_GO_LOW_POWER:
		state_queue->Push(ACT_SWITCH_NOMINAL);
		break;
	case MCB_GO_READY:
		state_queue->Push(ACT_SWITCH_READY);
		break;
	case MCB_HOME_LW:
		state_queue->Push(ACT_HOME_LW);
		break;
	case MCB_ZERO_REEL:
		state_queue->Push(ACT_ZERO_REEL);
		break;
	case MCB_GET_TEMPERATURES:
		monitor_queue->Push(MONITOR_SEND_TEMPS);
		break;
	case MCB_GET_VOLTAGES:
		monitor_queue->Push(MONITOR_SEND_VOLTS);
		break;
	case MCB_GET_CURRENTS:
		monitor_queue->Push(MONITOR_SEND_CURRS);
		break;
	case MCB_BRAKE_ON:
		state_queue->Push(ACT_BRAKE_ON);
		break;
	case MCB_BRAKE_OFF:
		state_queue->Push(ACT_BRAKE_OFF);
		break;
	case MCB_CONTROLLERS_ON:
		state_queue->Push(ACT_CONTROLLERS_ON);
		break;
	case MCB_CONTROLLERS_OFF:
		state_queue->Push(ACT_CONTROLLERS_OFF);
		break;
	case MCB_FULL_RETRACT:
		state_queue->Push(ACT_FULL_RETRACT);
		dibComm.TX_Ack(MCB_FULL_RETRACT, true);
		break;
	case MCB_IGNORE_LIMITS:
		state_queue->Push(ACT_IGNORE_LIMITS);
		dibComm.TX_Ack(MCB_IGNORE_LIMITS, true);
		break;
	case MCB_USE_LIMITS:
		state_queue->Push(ACT_USE_LIMITS);
		dibComm.TX_Ack(ACT_USE_LIMITS, true);
		break;
	// messages that have parameters to parse -------------
	case MCB_REEL_OUT:
		if (dibComm.RX_Reel_Out(&(mcbParameters.deploy_length), &(mcbParameters.deploy_velocity))) {
			state_queue->Push(ACT_SET_DEPLOY_V);
			state_queue->Push(ACT_DEPLOY_X);
		}
		break;
	case MCB_REEL_IN:
		if (dibComm.RX_Reel_In(&(mcbParameters.retract_length), &(mcbParameters.retract_velocity))) {
			state_queue->Push(ACT_SET_RETRACT_V);
			state_queue->Push(ACT_RETRACT_X);
		}
		break;
	case MCB_DOCK:
		if (dibComm.RX_Dock(&(mcbParameters.dock_length), &(mcbParameters.dock_velocity))) {
			state_queue->Push(ACT_SET_DOCK_V);
			state_queue->Push(ACT_DOCK);
		}
		break;
	case MCB_IN_NO_LW:
		if (dibComm.RX_In_No_LW(&(mcbParameters.retract_length), &(mcbParameters.retract_velocity))) {
			state_queue->Push(ACT_SET_RETRACT_V);
			state_queue->Push(ACT_IN_NO_LW);
		}
		break;
	case MCB_OUT_ACC:
		if (dibComm.RX_Out_Acc(&(mcbParameters.deploy_acceleration))) {
			state_queue->Push(ACT_SET_DEPLOY_A);
		}
		break;
	case MCB_IN_ACC:
		if (dibComm.RX_In_Acc(&(mcbParameters.retract_acceleration))) {
			state_queue->Push(ACT_SET_RETRACT_A);
		}
		break;
	case MCB_DOCK_ACC:
		if (dibComm.RX_Dock_Acc(&(mcbParameters.dock_acceleration))) {
			state_queue->Push(ACT_SET_DOCK_A);
		}
		break;
	case MCB_TEMP_LIMITS:
		if (dibComm.RX_Temp_Limits(&(mcbParameters.temp_limits[0]),&(mcbParameters.temp_limits[1]),&(mcbParameters.temp_limits[2]),&(mcbParameters.temp_limits[3]),&(mcbParameters.temp_limits[4]),&(mcbParameters.temp_limits[5]))) {
			state_queue->Push(ACT_TEMP_LIMITS);
		} else {
			Serial.println("Error setting temp limits");
		}
		break;
	case MCB_TORQUE_LIMITS:
		if (dibComm.RX_Torque_Limits(&(mcbParameters.torque_limits[0]),&(mcbParameters.torque_limits[1]))) {
			state_queue->Push(ACT_TORQUE_LIMITS);
		} else {
			Serial.println("Error setting torque limits");
		}
		break;
	case MCB_CURR_LIMITS:
		if (dibComm.RX_Curr_Limits(&(mcbParameters.curr_limits[0]),&(mcbParameters.curr_limits[1]))) {
			state_queue->Push(ACT_CURR_LIMITS);
		} else {
			Serial.println("Error setting temp limits");
		}
		break;
	default:
		storageManager.LogSD("Unknown DIB RX message", ERR_DATA);
		break;
	}
}

void InternalSerialDriverMCB::PrintDebugMenu()
{
	Serial.println("---- Debug Menu ----");
	PrintDebugCommand(MCB_CANCEL_MOTION, ";\t(cancel motion)");
	PrintDebugCommand(MCB_GO_LOW_POWER, ";\t(low power)");
	PrintDebugCommand(MCB_GO_READY, ";\t(ready)");
	PrintDebugCommand(MCB_HOME_LW, ";\t(home level wind)");
	PrintDebugCommand(MCB_ZERO_REEL, ";\t(zero reel)");
	PrintDebugCommand(MCB_GET_TEMPERATURES, ";\t(get temps)");
	PrintDebugCommand(MCB_GET_VOLTAGES, ";\t(get volts)");
	PrintDebugCommand(MCB_GET_CURRENTS, ";\t(get currs)");
	PrintDebugCommand(MCB_BRAKE_ON, ";\t(brake on)");
	PrintDebugCommand(MCB_BRAKE_OFF, ";\t(brake off)");
	PrintDebugCommand(MCB_CONTROLLERS_ON, ";\t(controllers on)");
	PrintDebugCommand(MCB_CONTROLLERS_OFF, ";\t(controllers off)");
	PrintDebugCommand(MCB_FULL_RETRACT, ";\t(full retract)");
	Serial.println("--------------------");
	PrintDebugCommand(MCB_REEL_OUT, ",num_revs,speed;\t(reel out)");
	PrintDebugCommand(MCB_REEL_IN, ",num_revs,speed;\t(reel in)");
	PrintDebugCommand(MCB_DOCK, ",num_revs,speed;\t(dock)");
	PrintDebugCommand(MCB_IN_NO_LW, ",num_revs,speed;\t(reel in without LW)");
	PrintDebugCommand(MCB_OUT_ACC, ",acc;\t(out acceleration)");
	PrintDebugCommand(MCB_IN_ACC, ",acc;\t(in acceleration)");
	PrintDebugCommand(MCB_DOCK_ACC, ",acc;\t(dock acceleration)");
	PrintDebugCommand(MCB_TEMP_LIMITS, ",mtr1hi,mtr1lo,mtr2hi,mtr2lo,mc1hi,mc1lo;\t(temp limits)");
	PrintDebugCommand(MCB_TORQUE_LIMITS, ",reel_hi,reel_lo;\t(torque limits)");
	PrintDebugCommand(MCB_CURR_LIMITS,",reel_hi,reel_lo\t(current limits)");
	Serial.println("--------------------");
}

void InternalSerialDriverMCB::PrintDebugCommand(uint8_t cmd, const char * description)
{
	Serial.print("#");
	Serial.print(cmd);
	Serial.println(description);
	delay(1);
}