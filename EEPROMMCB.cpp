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
	, mtr1_temp_lim({80.0f,-15.0f})
	, mtr2_temp_lim({60.0f,-15.0f})
	, mc1_temp_lim({80.0f,-40.0f})
	, mc2_temp_lim({FLT_MAX,FLT_MIN})      // limit not in use
	, dcdc_temp_lim({FLT_MAX,FLT_MIN})     // limit not in use
	, spare_therm_lim({FLT_MAX,FLT_MIN})   // limit not in use
	, vmon_3v3_lim({FLT_MAX,FLT_MIN})      // limit not in use
	, vmon_15v_lim({FLT_MAX,FLT_MIN})      // limit not in use
	, vmon_20v_lim({FLT_MAX,FLT_MIN})      // limit not in use
	, vmon_spool_lim({FLT_MAX,FLT_MIN})    // limit not in use
	, imon_brake_lim({FLT_MAX,FLT_MIN})    // limit not in use
	, imon_mc_lim({FLT_MAX,FLT_MIN})       // limit not in use
	, imon_mtr1_lim({13.75f,-10.0f})
	, imon_mtr2_lim({FLT_MAX,FLT_MIN})     // limit not in use
	, reel_torque_lim({500.0f,-500.0f})
	, lw_torque_lim({2000.0f,-2000.0f})
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
	success &= Register(&mtr1_temp_lim);
	success &= Register(&mtr2_temp_lim);
	success &= Register(&mc1_temp_lim);
	success &= Register(&mc2_temp_lim);
	success &= Register(&dcdc_temp_lim);
	success &= Register(&spare_therm_lim);
	success &= Register(&vmon_3v3_lim);
	success &= Register(&vmon_15v_lim);
	success &= Register(&vmon_20v_lim);
	success &= Register(&vmon_spool_lim);
	success &= Register(&imon_brake_lim);
	success &= Register(&imon_mc_lim);
	success &= Register(&imon_mtr1_lim);
	success &= Register(&imon_mtr2_lim);
	success &= Register(&reel_torque_lim);
	success &= Register(&lw_torque_lim);
	success &= Register(&tmslow_num_samples);
	success &= Register(&tmfast_num_samples);

    if (!success) {
        Serial.println("Error registering EEPROM configs");
    }
}