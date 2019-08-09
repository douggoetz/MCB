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
    ACT_HOME_LW,
    ACT_BRAKE_ON,
    ACT_BRAKE_OFF,

    // parameters to set
    ACT_SET_DEPLOY_V,
    ACT_SET_RETRACT_V,
    ACT_SET_DOCK_V,
    ACT_SET_DEPLOY_A,
    ACT_SET_RETRACT_A,
    ACT_SET_DOCK_A,
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

// Enum describing monitor actions
enum Monitor_Command_t : uint8_t {
    MONITOR_BOTH_MOTORS_ON,
    MONITOR_REEL_ON,
    MONITOR_MOTORS_OFF,
    MONITOR_LOW_POWER,
    MONITOR_SEND_TEMPS,
    MONITOR_SEND_VOLTS,
    MONITOR_SEND_CURRS,
    UNUSED_COMMAND
};

#endif