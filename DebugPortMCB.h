/*
 *  DebugPortMCB.h
 *  File defining the MCB debug port
 *  Author: Alex St. Clair
 *  September 2018
 */

#ifndef DEBUGPORTMCB_H_
#define DEBUGPORTMCB_H_

#include "ActionsMCB.h"
#include "InternalSerialDriverMCB.h"
#include "Queue.h"
#include "WProgram.h"
#include <String.h>
#include <StdInt.h>
#include <StdLib.h>

#define USER_INPUT_TIMEOUT  30

class DebugPortMCB {
public:
	DebugPortMCB(Queue * state_q, InternalSerialDriverMCB * dibdriver);
	~DebugPortMCB(void) { };
    void RunDebugPort(void);

private:
    void PrintMenu(void);
    bool GetUserCommand(void);
    void HandleUserCommand(void);
    void ExecuteCommand(void);
    void GetUserParameter(void);

    Queue * action_queue;
    InternalSerialDriverMCB * dibDriver;

    float user_float;
    char current_command;
    bool command_ready;
    bool waiting_on_input;
    bool startup;

};

#endif /* DEBUGPORTMCB_H_ */