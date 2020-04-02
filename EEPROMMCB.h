/*
 *  EEPROMMCB.h
 *  Author:  Alex St. Clair
 *  Created: April 2020
 *
 *  This class manages configuration storage in EEPROM on the MCB
 */

#ifndef EEPROMMCB_H
#define EEPROMMCB_H

#include "TeensyEEPROM.h"

class EEPROMMCB : public TeensyEEPROM {
private:
    void RegisterAll();

public:
    EEPROMMCB();

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
	EEPROMData<float> mtr1_temp_hi;
	EEPROMData<float> mtr1_temp_lo;
	EEPROMData<float> mtr2_temp_hi;
	EEPROMData<float> mtr2_temp_lo;
	EEPROMData<float> mc1_temp_hi;
	EEPROMData<float> mc1_temp_lo;
	EEPROMData<float> mc2_temp_hi;
	EEPROMData<float> mc2_temp_lo;
	EEPROMData<float> dcdc_temp_hi;
	EEPROMData<float> dcdc_temp_lo;
	EEPROMData<float> spare_therm_hi;
	EEPROMData<float> spare_therm_lo;

	// voltage monitor limits
	EEPROMData<float> vmon_3v3_hi;
	EEPROMData<float> vmon_3v3_lo;
	EEPROMData<float> vmon_15v_hi;
	EEPROMData<float> vmon_15v_lo;
	EEPROMData<float> vmon_20v_hi;
	EEPROMData<float> vmon_20v_lo;
	EEPROMData<float> vmon_spool_hi;
	EEPROMData<float> vmon_spool_lo;

	// current monitor limits
	EEPROMData<float> imon_brake_hi;
	EEPROMData<float> imon_brake_lo;
	EEPROMData<float> imon_mc_hi;
	EEPROMData<float> imon_mc_lo;
	EEPROMData<float> imon_mtr1_hi;
	EEPROMData<float> imon_mtr1_lo;
	EEPROMData<float> imon_mtr2_hi;
	EEPROMData<float> imon_mtr2_lo;

	// torque limits
	EEPROMData<float> reel_torque_hi;
	EEPROMData<float> reel_torque_lo;
	EEPROMData<float> lw_torque_hi;
	EEPROMData<float> lw_torque_lo;

	// telemetry sample averaging numbers
	EEPROMData<uint8_t> tmslow_num_samples;
	EEPROMData<uint8_t> tmfast_num_samples;

    // ----------------------------------------------------
};

#endif /* EEPROMMCB_H */