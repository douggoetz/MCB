/*
 *  InternalSerialDriverMCB.cpp
 *  File implementing the serial driver for communicating with the
 *  main payload computer (DIB) from the MCB.
 *  Author: Alex St. Clair
 *  February 2018
 */

#include "InternalSerialDriverMCB.h"

InternalSerialDriverMCB::InternalSerialDriverMCB(Queue* state_q)
    : dibComm(&DIB_SERIAL)
    , storageManager()
{
    state_queue = state_q;
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
		dibComm.TX_Ack(MCB_GO_LOW_POWER, true);
		break;
	case MCB_HOME_LW:
		// todo: action
		break;
	case MCB_ZERO_REEL:
		// todo: action
		break;
	case MCB_GET_TEMPERATURES:
		// todo: send to monitor
		break;
	case MCB_GET_VOLTAGES:
		// todo: send to monitor
		break;
	case MCB_GET_CURRENTS:
		// todo: send to monitor
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
	default:
		storageManager.LogSD("Unknown DIB RX message", ERR_DATA);
		break;
	}
}
