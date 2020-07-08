# MCB

This repository contains the Motor Control Board (MCB) software for LASP [Stratéole 2](https://strat2.org/) reeldown instruments (RACHuTS and FLOATS). It is structured as an Arduino library for the Teensy 3.6 Arduino-compatible board based on an ARM Cortex-M4 microcontroller. On RACHuTS, the MCB is the secondary computer to the Profiler Interface Board (PIB), the software for which is maintained in [StratoPIB](https://github.com/dastcvi/StratoPIB). On FLOATS, the MCB is a secondary computer to the Data Interface Board (DIB), maintained in [StratoDIB](https://github.com/dastcvi/StratoDIB). *Note that after the engineering flight, the MCB and DIB/PIB were combined electrically into one standard PCB (named the Monitoring and Doing board, or MonDo), but the MCB, PIB, and DIB software have remained separate and largely unaltered by this change*.

## Software Development Environment

The MCB uses a [Teensy 3.6](https://www.sparkfun.com/products/14057) Arduino-compatible MCU board as its primary computer. Thus, its code is implemented for Arduino, meaning that all of this C++ code uses the Arduino drivers for the Teensy 3.6 and is compiled using the Arduino IDE with the [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) plug-in.

The Arduino main file can be found in `examples/MCB_Main.ino`. To compile and load MCB, open this main file in Arduino and follow the Teensyduino instructions.

*The MCB is known to work with Arduino 1.8.4 and Teensyduino 1.39, as well as with Arduino 1.8.11 and Teensyduino 1.51*

## Software Control Structure

The MCB software implements a state-based design using a cyclic executive loop running at 1 Hz using best-effort timing. Exceeding one second in a loop is fine for the system, but it can cause a skipped loop. A watchdog timer will reset the microcontroller if a loop exceeds 10 seconds. The loop timing is controlled in the Arduino main file, and the loop structure is implemented in the MCB `Loop()` function located in `MCB.cpp` and called from the main file. The order of operations in each loop is:

1. `dibDriver.RunDebugDriver()`: Check for and route messages on the debug interface
2. `dibDriver.RunDriver()`: Check for and route messages on the DIB/PIB interface
3. `PerformActions()`: Check for and handle actions on the "action queue"
4. `RunState()`: Call a function to perform state-specific actions
5. `limitMonitor.Monitor()`: Monitor MCB vitals against configurable limits

## DIB/PIB Communication

The communication between the MCB and DIB or PIB is identical, and managed by the `InternalSerialDriverMCB` class that is instantiated as the `dibDriver`. The interface itself is defined using a class that derives from [SerialComm](https://github.com/dastcvi/SerialComm), which is a simple, robust protocol for inter-Arduino serial communication. This deriving class is version-controlled in a separate repository: [MCBComm](https://github.com/dastcvi/MCBComm). In each loop, the `dibDriver` object handles as many messages as are available on this interface. This can involve changing the state of the MCB, changing a configuration, or placing an action on the action queue, discussed later. Many times, this will also include sending an acknowledgment message back to the DIB/PIB as well.

A critical message type to be aware of is the `MCB_ERROR` message that contains a string. This allows the MCB to send a string describing any error encountered to the DIB/PIB that will then be sent to the ground. Thus, this is an invaluable message for alerting the ground of any errors.

### Internal Serial Buffering

The `MCBComm` interface relies on the serial message buffering internal to the Teensy 3.6 core drivers (see the [explanation in SerialComm](https://github.com/dastcvi/SerialComm#aside-on-arduinos-internal-serial-buffering)). The `MCBBufferGuard.h` file contains macros that ensure that the buffers have been correctly set, otherwise the macros will throw a compile-time error. On any computer that uses a Teensy where buffers are updated or memory is limited, it is recommended that you use a buffer guard like this for every project.

## Debug Interface

To help with testing, the MCB implements the `MCBComm` interface on the USB serial port, allowing it to receive all of the same commands from a user at a computer. When connected to the instrument over USB serial, send `m` to the instrument to list the command menu. A checksum is not required at the end of messages sent over this interface. Additionally, the instrument outputs useful debug messages over this serial connection.

## Actions and the "Action Queue"

To facilitate communication between states and drivers as well as to schedule actions for the next loop, the MCB contains an "action queue". The action queue is simply a queue of enumerated actions (8-bit unsigned integers) that are defined in the `ActionsMCB.h` file. The queue instantiates the [SafeBuffer](https://github.com/dastcvi/SafeBuffer) library. Once per loop, the `PerformActions()` function in `MCB.cpp` will handle as many actions as are available in the action queue.

## State Machine

The MCB has the following states:

* `ST_READY`: ready for actions, the motion controllers may or may not be powered on
* `ST_NOMINAL`: low power mode waiting for any commands
* `ST_REEL_OUT`: reel out a specified amount
* `ST_REEL_IN`: reel in a specified amount, the level wind will perform camming
* `ST_DOCK`: reel in a specified amount, the level wind will seek the center position
* `ST_IN_NO_LW`: reel in a specified amount, the level wind will not move
* `ST_HOME_LW`: re-home the level wind (used only for testing)

Each state is implemented as a function in the `States.cpp` file that is called once per loop if it is the current state. Each state maintains substates, contained in the `substate` variable and defined in the `MCBSubstates_t` enum. There are two special substates defined for all states: `STATE_ENTRY = ENTRY_SUBSTATE` and `STATE_EXIT = EXIT_SUBSTATE`. When a new state is called, the old state's function is first called after setting `substate = EXIT_SUBSTATE`, and then the new state is called after setting `substate = ENTRY_SUBSTATE`. Thus, each state has a chance to exit gracefully in the event of an unexpected state change (for example all of the reel operation states will stop motion).

### Reel Operation States

I will not delve into the specifics of each state, as this can be found best by just reading through the code, but it is worth describing the concept of operations for a full profile (reel out, reel in, dock) in order to understand why the level wind and reel are tasked the way they are. The reel is simple, it goes in and out at configurable speeds (slower for docking). The level wind is more complicated as it is responsible for transversing back and forth across the reel to lay the fiber evenly. Testing on RACHuTS showed that the tension in the fiber caused the level wind to stall if it moves while the reel is stationary. Thus, the level wind sequence is carefully orchestrated to ensure that the reel is always moving when it is:

1. Reel out profile occurs with no level wind motion
2. Reel in motion starts
3. Level wind performs a wind out, which includes homing
4. Level wind performs a wind in when the wind out completes
5. Level wind continues to wind in/out until the reel in completes
6. Reel in completes
7. Dock starts
8. Level wind moves to center position
9. Level wind reaches center position and stops
10. Dock completes

There are a few important quirks to be aware of that make scheding and timing of these operations critical.

* A dock must happen after a reel in motion in which the level wind successfully homes (if a dock is commanded and the level wind hasn't homed, it will send an `MCB_ERROR` message stating this)
* On the first level wind "wind out" operation, it homes from wherever it was last left at the current level wind speed. This means it can take quite a few reel rotations to home. If the level wind hasn't completed homing before the reel in motion completes, it stops and a dock motion cannot be called until it has successfully homed.
* After a "wind in" operation and directly before a "wind out", the level wind is left right next to the homing switch. When a "wind out" is called, it must successfully home or face the stipulations listed in the last bullet point. For this case, however, it will only take a couple of seconds to home, so it is improbable that the reel operation will conclude during one of these "intermediate re-homes".

## Limit Monitoring

Limit monitoring on the MCB, implemented in the `MonitorMCB` class, serves two purposes. First, it checks sensor readings against configured limits, and second, it aggregates sensor data during motion profiles to send to the DIB/PIB for transmission to the ground. The class is instantiated as `limitMonitor` and is called at initialization, and once per loop via the `Monitor()` function. Each loop, the limit monitor checks sensor readings on a rotating basis because it would take too long to check each sensor every loop. If a motion profile is ongoing, the monitor will also gather torque and position information from the motion controllers.

Since the limit monitor must change its behavior based on the state of the instrument, it has a command structure that is also based on the [SafeBuffer](https://github.com/dastcvi/SafeBuffer) queue. This queue is called the monitor queue and commands are placed on it primarily by state functions and the action queue handler.

## Technosoft

The reel and the level wind both use [Technosoft motion controllers](https://technosoftmotion.com/en/intelligent-drives/). The reel uses an iPOS4808 and the level wind uses an iPOS3604, but the interface is consistent between the two. The MCB actually communicates exclusively with the reel controller over RS-232 serial, which in turn communicates with the level wind controller using a shared CAN bus. A separate Arduino library, [Technosoft](https://github.com/dastcvi/Technosoft), implements the interface with these Technosoft controllers as a base class from which the `Reel` and `LevelWind` classes are derived.

There is also software implemented on each of the controllers using the [Technosoft Motion Language (TML)](https://www.technosoftmotion.com/ESM-um-html/index.html?help_tml_basic_concepts.htm). The software is implemented as functions that are callable by the MCB over serial, and can be found in the `Technosoft_Projects` directory. The projects are Technosoft archives that can be opened in Technosoft's EasyMotion Studio program using `Project -> Restore...`. There are currently two RACHuTS versions:

* `Flight_System_V3_RACHUTS.m.zip`: was flown during the engineering test flight, but is no longer supported
* `Flight_System_V6_RACHUTS_EmCamVariable.m.zip`: updates the flight version to perform "emulated camming" instead of using Technosoft's software camming feature

### Reel

The reel can be controlled using the `ReelIn()` and `ReelOut()` functions, which will handle the interface to call TML function on the controller. The length, speed, and acceleration of a profile can all be controlled using these functions. There is also a brake on the reel that can be controlled via TML functions (called by `BrakeOn()` and `BrakeOff()`), but this is primarily used for testing, and is handled automatically in the `ReelIn()` and `ReelOut()` functions.

### Level Wind

The level wind motion is a little bit more complex, and corresponds to the specific type of reel operation that is ongoing. In reel out operations, the level wind is assumed to be positioned in the center, and is not moved. During reel in operations, the level wind undergoes emulated camming: that is it moves back and forth across the reel to evenly lay the fiber on the spool. In dock operations, the level wind moves to the center and stays there so that it is positioned for reeling back out. These specifics are handled in the state machine by calling certain level wind functions implemented in TML on the controller. For reeling in, the `WindIn` and `WindOut` functions are used to perform emulated camming. For docking, the `SetCenter()` function is used. For testing, the `Home()` function can be used.

The level wind emulated camming uses a variable speed based on the speed of the reel. The `LW_MM_PER_ROT` macro in `LevelWind.h` is the constant used to determine how much the level wind moves when the reel moves. To change the level wind movement in proportion to the reel, simply change this macro.

## Configuration

Just like on the PIB, important configurations are stored in EEPROM on the MCB. The EEPROM storage is maintained by the `ConfigManagerMCB` class, which derives from [TeensyEEPROM](https://github.com/dastcvi/TeensyEEPROM). This library is a wrapper for the core EEPROM library that protects against EEPROM failure. A hard-coded default for each configuration is maintained in FLASH memory, and a mutable runtime variable exists for each in RAM. Thus, if the EEPROM fails, the configurations can still be changed in RAM and will update to a default value on a processor reset. The configurations can be changed via telecommands. These configurations can be changed over the `MCBComm` interface.

## SD Storage

The MCB also provides a class that handles writing runtime data to the SD card: `StorageManagerMCB`. It creates and maintains a filesystem on the SD card that allows it to separate different types of runtime data: voltage, current, temperature, errors, and motion data.