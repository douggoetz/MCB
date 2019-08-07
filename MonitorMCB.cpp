/*
 *  StateManagerMCB.h
 *  File implementing the MCB monitor
 *  Author: Alex St. Clair
 *  October 2018
 */

#include "MonitorMCB.h"

MonitorMCB::MonitorMCB(Queue * monitor_q, Queue * action_q, Reel * reel_in, LevelWind * lw_in) :
    ltcManager(LTC_TEMP_CS_PIN, LTC_TEMP_RESET_PIN, THERM_SENSE_CH, RTD_SENSE_CH),
    storageManager()
{
    monitor_low_power = false;
    monitor_reel = false;
    monitor_levelwind = false;
    monitor_queue = monitor_q;
    action_queue = action_q;
    reel = reel_in;
    levelWind = lw_in;
}

void MonitorMCB::InitializeSensors(void)
{
	analogReadRes(12);
	for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        ltcManager.channel_assignments[temp_sensors[i].channel_number] = temp_sensors[i].sensor_type;
    }
	ltcManager.InitializeAndConfigure();
}

void MonitorMCB::UpdateLimits(void)
{
    // temp sensor limits
    temp_sensors[MTR1_THERM].limit_hi = storageManager.eeprom_data.mtr1_temp_hi;
    temp_sensors[MTR1_THERM].limit_lo = storageManager.eeprom_data.mtr1_temp_lo;
    temp_sensors[MTR2_THERM].limit_hi = storageManager.eeprom_data.mtr2_temp_hi;
    temp_sensors[MTR2_THERM].limit_lo = storageManager.eeprom_data.mtr2_temp_lo;
    temp_sensors[MC1_THERM].limit_hi = storageManager.eeprom_data.mc1_temp_hi;
    temp_sensors[MC1_THERM].limit_lo = storageManager.eeprom_data.mc1_temp_lo;
    temp_sensors[MC2_THERM].limit_hi = storageManager.eeprom_data.mc2_temp_hi;
    temp_sensors[MC2_THERM].limit_lo = storageManager.eeprom_data.mc2_temp_lo;
    temp_sensors[DCDC_THERM].limit_hi = storageManager.eeprom_data.dcdc_temp_hi;
    temp_sensors[DCDC_THERM].limit_lo = storageManager.eeprom_data.dcdc_temp_lo;
    temp_sensors[SPARE_THERM].limit_hi = storageManager.eeprom_data.spare_therm_hi;
    temp_sensors[SPARE_THERM].limit_lo = storageManager.eeprom_data.spare_therm_lo;

    // vmon limits
    vmon_channels[VMON_3V3].limit_hi = storageManager.eeprom_data.vmon_3v3_hi;
    vmon_channels[VMON_3V3].limit_lo = storageManager.eeprom_data.vmon_3v3_lo;
    vmon_channels[VMON_15V].limit_hi = storageManager.eeprom_data.vmon_15v_hi;
    vmon_channels[VMON_15V].limit_lo = storageManager.eeprom_data.vmon_15v_lo;
    vmon_channels[VMON_20V].limit_hi = storageManager.eeprom_data.vmon_20v_hi;
    vmon_channels[VMON_20V].limit_lo = storageManager.eeprom_data.vmon_20v_lo;
    vmon_channels[VMON_SPOOL].limit_hi = storageManager.eeprom_data.vmon_spool_hi;
    vmon_channels[VMON_SPOOL].limit_lo = storageManager.eeprom_data.vmon_spool_lo;

    // imon limits
    imon_channels[IMON_BRK].limit_hi = storageManager.eeprom_data.imon_brake_hi;
    imon_channels[IMON_BRK].limit_lo = storageManager.eeprom_data.imon_brake_lo;
    imon_channels[IMON_MC].limit_hi = storageManager.eeprom_data.imon_mc_hi;
    imon_channels[IMON_MC].limit_lo = storageManager.eeprom_data.imon_mc_lo;
    imon_channels[IMON_MTR1].limit_hi = storageManager.eeprom_data.imon_mtr1_hi;
    imon_channels[IMON_MTR1].limit_lo = storageManager.eeprom_data.imon_mtr1_lo;
    imon_channels[IMON_MTR2].limit_hi = storageManager.eeprom_data.imon_mtr2_hi;
    imon_channels[IMON_MTR2].limit_lo = storageManager.eeprom_data.imon_mtr2_lo;

    // torque limits
    motor_torques[REEL_INDEX].limit_hi = storageManager.eeprom_data.reel_torque_hi;
    motor_torques[REEL_INDEX].limit_lo = storageManager.eeprom_data.reel_torque_lo;
    motor_torques[LEVEL_WIND_INDEX].limit_hi = storageManager.eeprom_data.lw_torque_hi;
    motor_torques[LEVEL_WIND_INDEX].limit_lo = storageManager.eeprom_data.lw_torque_lo;
}

void MonitorMCB::Monitor(void)
{
    static uint32_t curr_time = 0;
    static uint32_t last_temp = 0;
    static uint32_t last_adc = 0;

    HandleCommands();

    bool limits_ok = true;
    curr_time = millis();

    if (!monitor_low_power && (curr_time - last_temp) > TEMP_PERIOD) {
        //limits_ok &= CheckTemperatures();
        last_temp = curr_time;
    }

    if ((curr_time - last_adc) > ADC_PERIOD) {
        limits_ok &= CheckVoltages();
        last_adc = curr_time;
    }

    limits_ok &= CheckCurrents();

    if (monitor_reel || monitor_levelwind) {
        limits_ok &= CheckTorques();
        UpdatePositions();
        PrintMotorData();
    }
    
    if (!limits_ok) {
        action_queue->Push(ACT_LIMIT_EXCEEDED); // notify state manager
    }
}

void MonitorMCB::HandleCommands(void)
{
	uint8_t command;
	while (!monitor_queue->IsEmpty()) {
		command = UNUSED_COMMAND;
		if (!monitor_queue->Pop(&command)) {
			return;
		}

        switch (command) {
        case MONITOR_BOTH_MOTORS_ON:
            monitor_reel = true;
            monitor_levelwind = true;
            if (monitor_low_power) {
                ltcManager.WakeUp();
                monitor_low_power = false;
            }
            break;
        case MONITOR_REEL_ON:
            monitor_reel = true;
            monitor_levelwind = false;
            if (monitor_low_power) {
                ltcManager.WakeUp();
                monitor_low_power = false;
            }
            break;
        case MONITOR_MOTORS_OFF:
            monitor_reel = false;
            monitor_levelwind = false;
            if (monitor_low_power) {
                ltcManager.WakeUp();
                monitor_low_power = false;
            }
            break;
        case MONITOR_LOW_POWER:
            ltcManager.Sleep();
            monitor_low_power = true;
            monitor_reel = false;
            monitor_levelwind = false;
            break;
        case UNUSED_COMMAND:
        default:
            storageManager.LogSD("Unknown monitor queue error", ERR_DATA);
            break;
        }
    }
}

bool MonitorMCB::CheckTemperatures(void)
{
    bool limits_ok = true;
    float temp = 0.0f;
    String temperature_string = "";

    for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
        // read channel
        temp = ltcManager.MeasureChannel(temp_sensors[i].channel_number);
        
        // validate reading, check limits if valid
        if (temp != TEMPERATURE_ERROR && temp != LTC_SENSOR_ERROR) {
            temp_sensors[i].last_temperature = temp;

            if (temp > temp_sensors[i].limit_hi) {
                if (!temp_sensors[i].over_temp) { // if newly over
                    storageManager.LogSD("Over temperature", ERR_DATA);
                    temp_sensors[i].over_temp = true;
                    temp_sensors[i].under_temp = false;
                }
                limits_ok = false;
            } else if (temp < temp_sensors[i].limit_lo) {
                if (!temp_sensors[i].under_temp) { // if newly under
                    storageManager.LogSD("Under temperature", ERR_DATA);
                    temp_sensors[i].over_temp = false;
                    temp_sensors[i].under_temp = true;
                }
                limits_ok = false;
            } else {
                temp_sensors[i].over_temp = false;
                temp_sensors[i].under_temp = false;
            }
        } else {
            temp_sensors[i].sensor_error = true;
        }

        temperature_string += String(temp) + ",";
    }
    
    storageManager.LogSD(temperature_string, TEMP_DATA);

    return limits_ok;
}

bool MonitorMCB::CheckVoltages(void)
{
    bool limits_ok = true;
    float raw = 0.0f;
    String voltage_string = "";

    for (int i = 0; i < NUM_VMON_CHANNELS; i++) {
        // read channel
        raw = analogRead(vmon_channels[i].channel_pin);

        // calculate voltage given the resistor divider network
        vmon_channels[i].last_voltage = VREF * (raw / MAX_ADC_READ) / vmon_channels[i].voltage_divider;

        // check limits
        if (vmon_channels[i].last_voltage > vmon_channels[i].limit_hi) {
            if (!vmon_channels[i].over_voltage) { // if newly over
                storageManager.LogSD("Over voltage", ERR_DATA);
                vmon_channels[i].over_voltage = true;
                vmon_channels[i].under_voltage = false;
            }
            limits_ok = false;
        } else if (vmon_channels[i].last_voltage < vmon_channels[i].limit_lo) {
            if (!vmon_channels[i].under_voltage) { // if newly under
                storageManager.LogSD("Under voltage", ERR_DATA);
                vmon_channels[i].over_voltage = false;
                vmon_channels[i].under_voltage = true;
            }
            limits_ok = false;
        } else {
            vmon_channels[i].over_voltage = false;
            vmon_channels[i].under_voltage = false;
        }

        voltage_string += String(vmon_channels[i].last_voltage) + ",";
    }

    storageManager.LogSD(voltage_string, VMON_DATA);

    return limits_ok;
}

bool MonitorMCB::CheckCurrents(void)
{
    bool limits_ok = true;
    float raw = 0.0f;
    String current_string = "";

    for (int i = 0; i < NUM_IMON_CHANNELS; i++) {
        // read channel
        raw = analogRead(imon_channels[i].channel_pin);

        // calculate load current from sense current pin voltage (very sensitive to constants)
        imon_channels[i].last_current = SENSE_CURR_SLOPE * 
                ((VREF / imon_channels[i].pulldown_res) * (raw / MAX_ADC_READ) - I_OFFSET);

        // check limits
        if (imon_channels[i].last_current > imon_channels[i].limit_hi) {
            if (!imon_channels[i].over_current) { // if newly over
                storageManager.LogSD("Over current", ERR_DATA);
                imon_channels[i].over_current = true;
                imon_channels[i].under_current = false;
            }
            limits_ok = false;
        } else if (imon_channels[i].last_current < imon_channels[i].limit_lo) {
            if (!imon_channels[i].under_current) { // if newly under
                storageManager.LogSD("Under current", ERR_DATA);
                imon_channels[i].over_current = false;
                imon_channels[i].under_current = true;
            }
            limits_ok = false;
        } else {
            imon_channels[i].over_current = false;
            imon_channels[i].under_current = false;
        }

        current_string += String(imon_channels[i].last_current) + ",";
    }

    storageManager.LogSD(current_string, IMON_DATA);

    return limits_ok;
}

bool MonitorMCB::CheckTorques(void)
{
    bool limits_ok = true;
    String current_string = "";

    if (monitor_reel) {
        if (reel->ReadTorqueCurrent()) {
            motor_torques[REEL_INDEX].read_error = false;
            motor_torques[REEL_INDEX].last_torque = reel->torque_current / motor_torques[REEL_INDEX].conversion;

            if (motor_torques[REEL_INDEX].last_torque > motor_torques[REEL_INDEX].limit_hi) {
                if (!motor_torques[REEL_INDEX].over_torque) { // if newly over
                    storageManager.LogSD("Over torque, reel", ERR_DATA);
                    motor_torques[REEL_INDEX].over_torque = true;
                    motor_torques[REEL_INDEX].under_torque = false;
                }
                limits_ok = false;
            } else if (motor_torques[REEL_INDEX].last_torque < motor_torques[REEL_INDEX].limit_lo) {
                if (!motor_torques[REEL_INDEX].under_torque) { // if newly under
                    storageManager.LogSD("Under torque, reel", ERR_DATA);
                    motor_torques[REEL_INDEX].over_torque = false;
                    motor_torques[REEL_INDEX].under_torque = true;
                }
                limits_ok = false;
            } else {
                motor_torques[REEL_INDEX].over_torque = false;
                motor_torques[REEL_INDEX].under_torque = false;
            }
        } else {
            // TODO: error handling
            motor_torques[REEL_INDEX].read_error = true;
        }
    }

    if (monitor_levelwind) {
        if (levelWind->ReadTorqueCurrent()) {
            motor_torques[LEVEL_WIND_INDEX].read_error = false;
            motor_torques[LEVEL_WIND_INDEX].last_torque = levelWind->torque_current / motor_torques[LEVEL_WIND_INDEX].conversion;

            if (motor_torques[LEVEL_WIND_INDEX].last_torque > motor_torques[LEVEL_WIND_INDEX].limit_hi) {
                if (!motor_torques[LEVEL_WIND_INDEX].over_torque) { // if newly over
                    storageManager.LogSD("Over torque, level wind", ERR_DATA);
                    motor_torques[LEVEL_WIND_INDEX].over_torque = true;
                    motor_torques[LEVEL_WIND_INDEX].under_torque = false;
                }
                limits_ok = false;
            } else if (motor_torques[LEVEL_WIND_INDEX].last_torque < motor_torques[LEVEL_WIND_INDEX].limit_lo) {
                if (!motor_torques[LEVEL_WIND_INDEX].under_torque) { // if newly under
                    storageManager.LogSD("Under torque, level wind", ERR_DATA);
                    motor_torques[LEVEL_WIND_INDEX].over_torque = false;
                    motor_torques[LEVEL_WIND_INDEX].under_torque = true;
                }
                limits_ok = false;
            } else {
                motor_torques[LEVEL_WIND_INDEX].over_torque = false;
                motor_torques[LEVEL_WIND_INDEX].under_torque = false;
            }
        } else {
            // TODO: error handling
            motor_torques[LEVEL_WIND_INDEX].read_error = true;
        }
    }

    return limits_ok;
}

void MonitorMCB::UpdatePositions(void)
{
    if (monitor_reel && !reel->UpdatePosition()) {
		storageManager.LogSD("Error updating reel position", ERR_DATA);
	}

	if (monitor_levelwind && !levelWind->UpdatePosition()) {
		storageManager.LogSD("Error updating level wind position", ERR_DATA);
	}
}

void MonitorMCB::PrintMotorData(void)
{
	String data_string = "#data,";
	data_string += String(reel->absolute_position / REEL_UNITS_PER_REV);
	data_string += String(",");
	data_string += String(levelWind->absolute_position);
	data_string += String(",");
	data_string += String(motor_torques[REEL_INDEX].last_torque);
	data_string += String(",");
    if (monitor_levelwind) {
	    data_string += String(motor_torques[LEVEL_WIND_INDEX].last_torque);
    } else {
        data_string += String("0");
    }
	data_string += String(",");
	data_string += String(temp_sensors[1].last_temperature); // stepper motor temp
	data_string += String(",");
	data_string += String(temp_sensors[0].last_temperature); // brushless dc motor temp
	Serial.println(data_string);
	storageManager.LogSD(data_string, MOTION_DATA);
}