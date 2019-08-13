/*
 *  HardwareMCB.h
 *  Centralized hardware definitions for the motor control board
 *  Author: Alex St. Clair
 *  Created: December, 2017
 *
 *  This file defines Teensy pins and other hardware specifications
 *  relevant to the motor control board.
 */

#ifndef HARDWAREMCB_H
#define HARDWAREMCB_H

#include <stdint.h>

// Which instrument?
#define INST_RACHUTS
//#define INST_FLOATS

// Pin definitions
#define MC1_IN1_PIN				2
#define MC1_IN4_PIN				3
#define MC2_IN1_PIN				4
#define MTR1_ENABLE_PIN			5
#define MTR2_ENABLE_PIN			6
#define ONBOARD_LED_PIN			13
#define MC1_ENABLE_PIN			14
#define LTC_TEMP_CS_PIN			15
#define MC2_ENABLE_PIN			17
#define MC1_OUT1_ERROR_PIN		18
#define MC1_OUT3_RDY_PIN		19
#define MC_ENABLE_PIN			22
#define MC2_OUT2_ERROR_PIN		23
#define MC2_OUT3_RDY_PIN		24
#define LTC_TEMP_RESET_PIN		28
#define FORCEON_PIN				29
#define FORCEOFF_PIN			30
#define BRAKE_ENABLE_PIN		33
#define PULSE_LED_PIN			38

// Analog pin definitions
#define A_VMON_3V3				A7
//#define A_VMON_12V			A6	// unused
#define A_VMON_15V				A2
#define A_VMON_20V				A20
#define A_IMON_BRK				A16
#define A_IMON_MC				A17
#define A_IMON_MTR1				A11
#define A_IMON_MTR2				A10
#define A_SPOOL_LEVEL			A22

// VMON indices (for MonitorMCB table)
enum VMON_Indices_t {
	VMON_3V3 = 0,
	VMON_15V,
	VMON_20V,
	VMON_SPOOL,
	NUM_VMON_CHANNELS
};

// IMON indices (for MonitorMCB table)
enum IMON_Indices_t {
	IMON_BRK = 0,
	IMON_MC,
	IMON_MTR1,
	IMON_MTR2,
	NUM_IMON_CHANNELS
};

// LTC2983 channel definitions
#define RTD_SENSE_CH			0 // no RTD sense
#define THERM_SENSE_CH			2
#define MTR1_THERM_CH			4
#define MTR2_THERM_CH			6
#define MC1_THERM_CH			8
#define MC2_THERM_CH			10
#define DCDC_THERM_CH			12
#define SPARE_THERM_CH			14

// Temp sensor indices (for MonitorMCB table)
enum Temp_Sensor_Indices_t {
	MTR1_THERM = 0,
	MTR2_THERM,
	MC1_THERM,
	MC2_THERM,
	DCDC_THERM,
	SPARE_THERM,
	NUM_TEMP_SENSORS
};

// Port definitions
#define DEBUG_SERIAL			Serial
#define DIB_SERIAL				Serial1
#define MC1_SERIAL				Serial2
#define MC2_SERIAL				Serial3
#define INVALID_PORT			Serial6

// ADC defs
#define MAX_ADC_READ			4095
#define VREF					3.196f
#define SENSE_CURR_SLOPE		11700.0f   // calibrated for the flight RACHuTS reel switch only!
#define I_OFFSET				0.00018f   // calibrated for the flight RACHuTS reel switch only!
#define RES_2K					2000.0f
#define RES_15K					15000.0f

// EEPROM definitions
#define EEPROM_BASE_ADDRESS		0
#define EEPROM_MAX_ADDRESS		4095
#define EEPROM_VERSION			(uint32_t) 0xA5C60001

// EEPROM data layout, note: will not pack, so try to arrange in 32-bit blocks
struct EEPROM_Data_t {
	// avoid configure mismatches
	uint32_t eeprom_version; // update every time the struct is changed

	// track reboots
	uint32_t boot_count;

	// versioning
	char hardware_version[2];
	uint8_t software_version;
	uint8_t serial_number;

	// default parameters
	float deploy_velocity;
	float deploy_acceleration;
	float retract_velocity;
	float retract_acceleration;
	float dock_velocity;
	float dock_acceleration;

	// temperature limits
	float mtr1_temp_hi;
	float mtr1_temp_lo;
	float mtr2_temp_hi;
	float mtr2_temp_lo;
	float mc1_temp_hi;
	float mc1_temp_lo;
	float mc2_temp_hi;
	float mc2_temp_lo;
	float dcdc_temp_hi;
	float dcdc_temp_lo;
	float spare_therm_hi;
	float spare_therm_lo;

	// voltage monitor limits
	float vmon_3v3_hi;
	float vmon_3v3_lo;
	float vmon_15v_hi;
	float vmon_15v_lo;
	float vmon_20v_hi;
	float vmon_20v_lo;
	float vmon_spool_hi;
	float vmon_spool_lo;

	// current monitor limits
	float imon_brake_hi;
	float imon_brake_lo;
	float imon_mc_hi;
	float imon_mc_lo;
	float imon_mtr1_hi;
	float imon_mtr1_lo;
	float imon_mtr2_hi;
	float imon_mtr2_lo;

	// torque limits
	float reel_torque_hi;
	float reel_torque_lo;
	float lw_torque_hi;
	float lw_torque_lo;
};

#endif
