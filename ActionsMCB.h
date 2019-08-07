/*
 *  ActionsMCB.h
 *  File defining actions the MCB can take
 *  Author: Alex St. Clair
 *  August 2019
 */

#ifndef ACTIONSMCB_H
#define ACTIONSMCB_H

#include <stdint.h>

// Enum describing state machine actions
enum State_Actions_t : uint8_t {
	// mode switches
	ACT_SWITCH_READY = 0,
	ACT_SWITCH_NOMINAL,

	// reel commands
	ACT_DEPLOY_X,
	ACT_RETRACT_X,
	ACT_DOCK,
	ACT_BRAKE_ON,
	ACT_BRAKE_OFF,

	// parameters to set
	ACT_SET_DV,
	ACT_SET_RV,
	ACT_SET_DA,
	ACT_SET_RA,
	ACT_SET_SERIAL,
	ACT_ZERO_REEL,

	// values to send
	ACT_SEND_POSITION,
	ACT_SEND_BRAKE_STATUS,
	ACT_SEND_STATE,
	ACT_SEND_CURRENT,

	// other
	ACT_LIMIT_EXCEEDED,

	ACT_NUM_ACTIONS, // not an action, used for counting
	ACT_UNUSED, // not an action, used as default
};

#endif