/*
 *  ConfigManagerMCB.h
 *  Author:  Alex St. Clair
 *  Created: April 2020
 *
 *  This class manages configuration storage in EEPROM on the MCB
 */

#ifndef CONFIGMANAGERMCB_H
#define CONFIGMANAGERMCB_H

#include "TeensyEEPROM.h"

struct Limit_Config_t {
    float hi;
    float lo;
};

class ConfigManagerMCB : public TeensyEEPROM {
private:
    void RegisterAll();

public:
    ConfigManagerMCB();

    // constants, manually change version number here to force update
    static const uint16_t CONFIG_VERSION = 0x5C00;
    static const uint16_t BASE_ADDRESS = 0x0000;

    // ------------------ Configurations ------------------

	// track reboots
	EEPROMData<uint32_t> boot_count;

	// default parameters
	EEPROMData<float> deploy_velocity;
	EEPROMData<float> deploy_acceleration;
	EEPROMData<float> retract_velocity;
	EEPROMData<float> retract_acceleration;
	EEPROMData<float> dock_velocity;
	EEPROMData<float> dock_acceleration;

	// temperature limits
	EEPROMData<Limit_Config_t> mtr1_temp_lim;
	EEPROMData<Limit_Config_t> mtr2_temp_lim;
	EEPROMData<Limit_Config_t> mc1_temp_lim;
	EEPROMData<Limit_Config_t> mc2_temp_lim;
	EEPROMData<Limit_Config_t> dcdc_temp_lim;
	EEPROMData<Limit_Config_t> spare_therm_lim;

	// voltage monitor limits
	EEPROMData<Limit_Config_t> vmon_3v3_lim;
	EEPROMData<Limit_Config_t> vmon_15v_lim;
	EEPROMData<Limit_Config_t> vmon_20v_lim;
	EEPROMData<Limit_Config_t> vmon_spool_lim;

	// current monitor limits
	EEPROMData<Limit_Config_t> imon_brake_lim;
	EEPROMData<Limit_Config_t> imon_mc_lim;
	EEPROMData<Limit_Config_t> imon_mtr1_lim;
	EEPROMData<Limit_Config_t> imon_mtr2_lim;

	// torque limits
	EEPROMData<Limit_Config_t> reel_torque_lim;
	EEPROMData<Limit_Config_t> lw_torque_lim;

	// telemetry sample averaging numbers
	EEPROMData<uint8_t> tmslow_num_samples;
	EEPROMData<uint8_t> tmfast_num_samples;

    // ----------------------------------------------------
};

#endif /* CONFIGMANAGERMCB_H */