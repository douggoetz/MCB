/*
 *  EEPROMMCB.cpp
 *  Author:  Alex St. Clair
 *  Created: April 2020
 *
 *  This class manages configuration storage in EEPROM on the MCB
 */

#include "EEPROMMCB.h"
#include "Reel.h"
#include <float.h>

EEPROMMCB::EEPROMMCB()
    : TeensyEEPROM(CONFIG_VERSION, BASE_ADDRESS)
    // ------------ Hard-Coded Config Defaults ------------
	, boot_count(0)
	, deploy_velocity(DEFAULT_FULL_SPEED)
	, deploy_acceleration(DEFAULT_ACC)
	, retract_velocity(DEFAULT_FULL_SPEED)
	, retract_acceleration(DEFAULT_ACC)
	, dock_velocity(DEFAULT_DOCK_SPEED)
	, dock_acceleration(DEFAULT_ACC)
	, mtr1_temp_hi(80.0f)
	, mtr1_temp_lo(-15.0f)
	, mtr2_temp_hi(60.0f)
	, mtr2_temp_lo(-15.0f)
	, mc1_temp_hi(80.0f)
	, mc1_temp_lo(-40.0f)
	, mc2_temp_hi(FLT_MAX)      // limit not in use
	, mc2_temp_lo(FLT_MIN)      // limit not in use
	, dcdc_temp_hi(FLT_MAX)     // limit not in use
	, dcdc_temp_lo(FLT_MIN)     // limit not in use
	, spare_therm_hi(FLT_MAX)   // limit not in use
	, spare_therm_lo(FLT_MIN)   // limit not in use
	, vmon_3v3_hi(FLT_MAX)      // limit not in use
	, vmon_3v3_lo(FLT_MIN)      // limit not in use
	, vmon_15v_hi(FLT_MAX)      // limit not in use
	, vmon_15v_lo(FLT_MIN)      // limit not in use
	, vmon_20v_hi(FLT_MAX)      // limit not in use
	, vmon_20v_lo(FLT_MIN)      // limit not in use
	, vmon_spool_hi(FLT_MAX)    // limit not in use
	, vmon_spool_lo(FLT_MIN)    // limit not in use
	, imon_brake_hi(FLT_MAX)    // limit not in use
	, imon_brake_lo(FLT_MIN)    // limit not in use
	, imon_mc_hi(FLT_MAX)       // limit not in use
	, imon_mc_lo(FLT_MIN)       // limit not in use
	, imon_mtr1_hi(13.75f)
	, imon_mtr1_lo(-10.0f)
	, imon_mtr2_hi(FLT_MAX)     // limit not in use
	, imon_mtr2_lo(FLT_MIN)     // limit not in use
	, reel_torque_hi(500.0f)
	, reel_torque_lo(-500.0f)
	, lw_torque_hi(2000.0f)
	, lw_torque_lo(-2000.0f)
	, tmslow_num_samples(60)
	, tmfast_num_samples(10)
    // ----------------------------------------------------
{ }


void EEPROMMCB::RegisterAll()
{
    bool success = true;

    success &= Register(&boot_count);
	success &= Register(&deploy_velocity);
	success &= Register(&deploy_acceleration);
	success &= Register(&retract_velocity);
	success &= Register(&retract_acceleration);
	success &= Register(&dock_velocity);
	success &= Register(&dock_acceleration);
	success &= Register(&mtr1_temp_hi);
	success &= Register(&mtr1_temp_lo);
	success &= Register(&mtr2_temp_hi);
	success &= Register(&mtr2_temp_lo);
	success &= Register(&mc1_temp_hi);
	success &= Register(&mc1_temp_lo);
	success &= Register(&mc2_temp_hi);
	success &= Register(&mc2_temp_lo);
	success &= Register(&dcdc_temp_hi);
	success &= Register(&dcdc_temp_lo);
	success &= Register(&spare_therm_hi);
	success &= Register(&spare_therm_lo);
	success &= Register(&vmon_3v3_hi);
	success &= Register(&vmon_3v3_lo);
	success &= Register(&vmon_15v_hi);
	success &= Register(&vmon_15v_lo);
	success &= Register(&vmon_20v_hi);
	success &= Register(&vmon_20v_lo);
	success &= Register(&vmon_spool_hi);
	success &= Register(&vmon_spool_lo);
	success &= Register(&imon_brake_hi);
	success &= Register(&imon_brake_lo);
	success &= Register(&imon_mc_hi);
	success &= Register(&imon_mc_lo);
	success &= Register(&imon_mtr1_hi);
	success &= Register(&imon_mtr1_lo);
	success &= Register(&imon_mtr2_hi);
	success &= Register(&imon_mtr2_lo);
	success &= Register(&reel_torque_hi);
	success &= Register(&reel_torque_lo);
	success &= Register(&lw_torque_hi);
	success &= Register(&lw_torque_lo);
	success &= Register(&tmslow_num_samples);
	success &= Register(&tmfast_num_samples);

    if (!success) {
        Serial.println("Error registering EEPROM configs");
    }
}