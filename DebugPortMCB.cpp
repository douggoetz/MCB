/*
 *  DebugPortMCB.cpp
 *  File implementing the MCB debug port
 *  Author: Alex St. Clair
 *  September 2018
 */

#include "DebugPortMCB.h"


DebugPortMCB::DebugPortMCB(Queue * state_q, InternalSerialDriverMCB * dibdriver)
{
    action_queue = state_q;
    dibDriver = dibdriver;
    user_float = 0.0f;
    current_command = '\0';
    command_ready = false;
    waiting_on_input = false;
    startup = true;
}

void DebugPortMCB::PrintMenu(void)
{
    Serial.println("\n-- Command Menu --");
    Serial.println("'0': Reel out");
    Serial.println("'1': Reel in");
    Serial.println("'2': Dock");
    Serial.println("'3': Set mode nominal (low power)");
    Serial.println("'4': Set mode ready");
    Serial.println("'5': Brake on");
    Serial.println("'6': Brake off");
    Serial.println("'7': Zero reel position");
    Serial.println("'8': Enter deploy velocity");
    Serial.println("'9': Enter rectract velocity");
    Serial.println("'p'|'m': re-print menu");
    Serial.println("'c'|'s': stop/cancel motion immediately");
    Serial.println("------------------\n");
}

bool DebugPortMCB::GetUserCommand(void)
{
    if (!Serial.available()) return false;

    current_command = Serial.read();

    // clear the buffer
    while (Serial.available()) Serial.read();

    return true;
}

void DebugPortMCB::HandleUserCommand(void)
{
    switch (current_command) {
    case '0':
        Serial.println("DEBUG: reel out");
        Serial.println("DEBUG: enter the number of rotations");
        waiting_on_input = true;
        command_ready = false;
        break;
    case '1':
        Serial.println("DEBUG: reel in");
        Serial.println("DEBUG: enter the number of rotations");
        waiting_on_input = true;
        command_ready = false;
        break;
    case '2':
        Serial.println("DEBUG: dock");
        Serial.println("DEBUG: enter the number of rotations");
        waiting_on_input = true;
        command_ready = false;
        break;
    case '3':
        Serial.println("DEBUG: set mode nominal");
        waiting_on_input = false;
        command_ready = true;
        break;
    case '4':
        Serial.println("DEBUG: set mode ready");
        waiting_on_input = false;
        command_ready = true;
        break;
    case '5':
        Serial.println("DEBUG: brake on");
        waiting_on_input = false;
        command_ready = true;
        break;
    case '6':
        Serial.println("DEBUG: brake off");
        waiting_on_input = false;
        command_ready = true;
        break;
    case '7':
        Serial.println("DEBUG: zero reel position");
        waiting_on_input = false;
        command_ready = true;
        break;
    case '8':
        Serial.println("DEBUG: enter deploy velocity");
        waiting_on_input = true;
        command_ready = false;
        break;
    case '9':
        Serial.println("DEBUG: enter retract velocity");
        waiting_on_input = true;
        command_ready = false;
        break;
    case 'p': case 'm': case 'P': case 'M':
        waiting_on_input = false;
        command_ready = true;
        break;
    case 's': case 'c': case 'S': case 'C':
        Serial.println("DEBUG: stop/cancel motion immediately");
        waiting_on_input = false;
        command_ready = true;
        break;
    default:
        Serial.println("DEBUG: invalid command");
        current_command = '\0';
        break;
    }
}

void DebugPortMCB::GetUserParameter(void)
{
    static uint32_t user_input_start = 0;

    if (user_input_start == 0) {
        user_input_start = millis();
    }

    // time out after a minute
    if (millis() - user_input_start > 60000) {
        Serial.println("DEBUG: user input timeout");
        user_input_start = 0;
        waiting_on_input = false;
    }

    if (!Serial.available()) return;

    // currently all parameters are floats
    String in_string = "";
    char in_char = '\0';

    // read until a newline is reached or the buffer ends
    while (Serial.available() > 0) {
        in_char = Serial.read();
        in_string += in_char;

        if (in_char == '\n') break;
    }

    // clear the buffer
    while (Serial.available()) Serial.read();

    user_float = in_string.toFloat();

    user_input_start = 0;
    waiting_on_input = false;
    command_ready = true;
}

void DebugPortMCB::ExecuteCommand(void)
{
    switch (current_command) {
    case '0':
        dibDriver->mcbParameters.deploy_length = user_float;
        action_queue->Push(ACT_DEPLOY_X); // command reel in
        break;
    case '1':
        dibDriver->mcbParameters.retract_length = user_float;
        action_queue->Push(ACT_RETRACT_X); // command reel in
        break;
    case '2':
        dibDriver->mcbParameters.dock_length = user_float;
        action_queue->Push(ACT_DOCK); // command reel out
        break;
    case '3':
        action_queue->Push(ACT_SWITCH_NOMINAL);
        break;
    case '4':
        action_queue->Push(ACT_SWITCH_READY);
        break;
    case '5':
        action_queue->Push(ACT_BRAKE_ON);
        break;
    case '6':
        action_queue->Push(ACT_BRAKE_OFF);
        break;
    case '7':
        action_queue->Push(ACT_ZERO_REEL);
        break;
    case '8':
        dibDriver->mcbParameters.deploy_velocity = user_float;
        action_queue->Push(ACT_SET_DEPLOY_V);
        break;
    case '9':
        dibDriver->mcbParameters.retract_velocity = user_float;
        action_queue->Push(ACT_SET_RETRACT_V);
        break;
    case 'p': case 'm': case 'P': case 'M':
        PrintMenu();
        break;
    case 's': case 'c': case 'S': case 'C':
        // leaving the reel ops state will cancel any ongoing motion
        action_queue->Push(ACT_SWITCH_READY);
        break;
    default:
        Serial.println("DEBUG: invalid command (to execute)");
        break;
    }

    current_command = '\0';
    command_ready = false;
}

void DebugPortMCB::RunDebugPort(void)
{
    if (startup) {
        PrintMenu();
        startup = false;
    }

    if (command_ready) {
        ExecuteCommand();
    } else if (waiting_on_input) {
        GetUserParameter();
    } else if (GetUserCommand()) {
        HandleUserCommand();
    }
}