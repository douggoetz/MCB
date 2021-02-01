/*
 *  MCB.h
 *  File defining the MCB state manager
 *  Author: Alex St. Clair
 *  February 2018
 */

/*  To add a new state:
 *  1) Add it to the enum "MCB_States_t"
 *  2) Define a private member function of the form "void func(bool exit)""
 *  3) Add the state to the private array "state_array"
 *  4) Write the function for the state in the .cpp file
 */

#ifndef MCB_H_
#define MCB_H_

#include "MCBBufferGuard.h"
#include "InternalSerialDriverMCB.h"
//#include "LTC2983Manager.h"
#include "PowerControllerMCB.h"
#include "StorageManagerMCB.h"
#include "ConfigManagerMCB.h"
#include "LevelWind.h"
#include "MonitorMCB.h"
#include "ActionsMCB.h"
#include "Reel.h"
#include "SafeBuffer.h"
#include <String.h>
#include <StdInt.h>
#include <StdLib.h>

#define ENTRY_SUBSTATE	0
#define EXIT_SUBSTATE	1

// Enum describing all states
enum MCB_States_t : uint8_t {
	ST_READY = 0,
	ST_NOMINAL,
	ST_REEL_OUT,
	ST_REEL_IN,
	ST_DOCK,
	ST_IN_NO_LW,
	ST_HOME_LW,
	NUM_STATES, // not a state, used for counting
	UNUSED_STATE = 0xFF // not a state, used as default
};

class MCB {
public:
	MCB();
	~MCB(void) { };

	// public interface for MCB_Main.ino
	void Startup();
	void Loop();

private:
	// handle commanded actions
	void PerformActions(void);

	// state machine interface
	void RunState(void);
	bool SetState(MCB_States_t new_state);

	// Watchdog
    void InitializeWatchdog();
    void KickWatchdog();

	// State Methods, located in States.cpp file
	void Ready();
	void Nominal();
	void ReelOut();
	void ReelIn();
	void Dock();
	void InNoLW();
	void HomeLW();

	// Array of state functions
	void (MCB::*state_array[NUM_STATES])() = {
		&MCB::Ready,
		&MCB::Nominal,
		&MCB::ReelOut,
		&MCB::ReelIn,
		&MCB::Dock,
		&MCB::InNoLW,
		&MCB::HomeLW
	};

	// Helper functions
	void PrintBootInfo(void);
	void CheckReel(void);
	bool CheckLevelWind(void); // returns true iff motion complete
	void CheckLevelWindCam(void);
	void LogFault(void);

	// Reel and level wind power control
	bool ReelControllerOn(void);
	bool LevelWindControllerOn(void);
	void ReelControllerOff(void);
	void LevelWindControllerOff(void);

	// send EEPROM contents to DIB/PIB as binary message
	void SendEEPROM(void);

	// Interface objects
	SafeBuffer action_queue;
	SafeBuffer monitor_queue;

	uint8_t action_queue_buffer[16] = {0};
	uint8_t monitor_queue_buffer[16] = {0};

	// Hardware objects
	PowerControllerMCB powerController;
	StorageManagerMCB storageManager;
	ConfigManagerMCB configManager;

	// Serial interface objects
	InternalSerialDriverMCB dibDriver;

	// Reel and level wind objects
	Reel reel;
	LevelWind levelWind;

	// Limit monitor object
	MonitorMCB limitMonitor;

	// Variables tracking current and last state
	MCB_States_t curr_state = ST_READY;
	MCB_States_t last_state = ST_READY;

	// Maintain motor operation information
	uint32_t last_pos_print = 0;
	bool reel_initialized = false;
	bool levelwind_initialized = false;
	bool camming = false;
	bool homed = false;
	bool lw_docked = false;
	bool lw_direction_out = true;

	// The current substate
	uint8_t substate = ENTRY_SUBSTATE;

	// buffer used for sending EEPROM contents as TM
	uint8_t eeprom_buffer[MAX_MCB_BINARY];

};

#endif