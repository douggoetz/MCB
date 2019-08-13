/*
 *  StateManagerMCB.h
 *  File implementing the MCB monitor
 *  Author: Alex St. Clair
 *  October 2018
 */

#include "MonitorMCB.h"

MonitorMCB::MonitorMCB(Queue * monitor_q, Queue * action_q, Reel * reel_in, LevelWind * lw_in, InternalSerialDriverMCB * dibdriver)
    : ltcManager(LTC_TEMP_CS_PIN, LTC_TEMP_RESET_PIN, THERM_SENSE_CH, RTD_SENSE_CH)
    , storageManager()
{
    monitor_low_power = false;
    monitor_reel = false;
    monitor_levelwind = false;
    monitor_queue = monitor_q;
    action_queue = action_q;
    reel = reel_in;
    levelWind = lw_in;
    dibDriver = dibdriver;
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
    bool limits_ok = true;

    HandleCommands();

    limits_ok &= CheckVoltages();
    limits_ok &= CheckCurrents();
    if (!monitor_low_power) {
        limits_ok &= CheckTemperatures();
    }

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
        case MONITOR_SEND_TEMPS:
            if (monitor_low_power) {
                dibDriver->dibComm.TX_Temperatures(LTC_POWERED_OFF, LTC_POWERED_OFF, LTC_POWERED_OFF, LTC_POWERED_OFF, LTC_POWERED_OFF, LTC_POWERED_OFF);
            } else {
                dibDriver->dibComm.TX_Temperatures(temp_sensors[0].last_temperature, temp_sensors[1].last_temperature, temp_sensors[2].last_temperature,
                                                   temp_sensors[3].last_temperature, temp_sensors[4].last_temperature, temp_sensors[5].last_temperature);
            }
            break;
        case MONITOR_SEND_VOLTS:
            dibDriver->dibComm.TX_Voltages(vmon_channels[0].last_voltage, vmon_channels[1].last_voltage, vmon_channels[2].last_voltage, vmon_channels[3].last_voltage);
            break;
        case MONITOR_SEND_CURRS:
            dibDriver->dibComm.TX_Currents(imon_channels[0].last_current, imon_channels[1].last_current, imon_channels[2].last_current, imon_channels[3].last_current);
            break;
        case UNUSED_COMMAND:
        default:
            storageManager.LogSD("Unknown monitor queue error", ERR_DATA);
            break;
        }
    }
}

// measures one sensor at a time in a non-blocking fashion
bool MonitorMCB::CheckTemperatures(void)
{
    static String temperature_string = "";
    static uint8_t curr_sensor = 0;
    static bool measurement_ongoing = false;
    static uint32_t last_temp_log = 0;
    bool limits_ok = true;
    float temp = 0.0f;

    if (!measurement_ongoing) {
        ltcManager.StartMeasurement(temp_sensors[curr_sensor].channel_number);
        measurement_ongoing = true;
    } else if (ltcManager.FinishedMeasurement()) {
        measurement_ongoing = false;
        temp = ltcManager.ReadMeasurementResult(temp_sensors[curr_sensor].channel_number);

        // validate reading, check limits if valid
        if (temp != TEMPERATURE_ERROR && temp != LTC_SENSOR_ERROR) {
            temp_sensors[curr_sensor].last_temperature = temp;

            if (temp > temp_sensors[curr_sensor].limit_hi) {
                if (!temp_sensors[curr_sensor].over_temp) { // if newly over
                    storageManager.LogSD("Over temperature", ERR_DATA);
                    temp_sensors[curr_sensor].over_temp = true;
                    temp_sensors[curr_sensor].under_temp = false;
                }
                limits_ok = false;
            } else if (temp < temp_sensors[curr_sensor].limit_lo) {
                if (!temp_sensors[curr_sensor].under_temp) { // if newly under
                    storageManager.LogSD("Under temperature", ERR_DATA);
                    temp_sensors[curr_sensor].over_temp = false;
                    temp_sensors[curr_sensor].under_temp = true;
                }
                limits_ok = false;
            } else {
                temp_sensors[curr_sensor].over_temp = false;
                temp_sensors[curr_sensor].under_temp = false;
            }
        } else {
            temp_sensors[curr_sensor].sensor_error = true;
        }

        temperature_string += String(temp) + ",";

        if (++curr_sensor == NUM_TEMP_SENSORS) {
            curr_sensor = 0;
            if (millis() > last_temp_log + TEMP_LOG_PERIOD) {
                storageManager.LogSD(temperature_string, TEMP_DATA);
                last_temp_log = millis();
            }
            temperature_string = "";
        }
    }

    return limits_ok;
}

bool MonitorMCB::CheckVoltages(void)
{
    static String voltage_string = "";
    static uint8_t curr_channel = 0;
    static uint32_t last_volt_log = 0;
    bool limits_ok = true;
    float raw = 0.0f;

    // read channel
    raw = analogRead(vmon_channels[curr_channel].channel_pin);

    // calculate voltage given the resistor divider network
    vmon_channels[curr_channel].last_voltage = VREF * (raw / MAX_ADC_READ) / vmon_channels[curr_channel].voltage_divider;

    // check limits
    if (vmon_channels[curr_channel].last_voltage > vmon_channels[curr_channel].limit_hi) {
        if (!vmon_channels[curr_channel].over_voltage) { // if newly over
            storageManager.LogSD("Over voltage", ERR_DATA);
            vmon_channels[curr_channel].over_voltage = true;
            vmon_channels[curr_channel].under_voltage = false;
        }
        limits_ok = false;
    } else if (vmon_channels[curr_channel].last_voltage < vmon_channels[curr_channel].limit_lo) {
        if (!vmon_channels[curr_channel].under_voltage) { // if newly under
            storageManager.LogSD("Under voltage", ERR_DATA);
            vmon_channels[curr_channel].over_voltage = false;
            vmon_channels[curr_channel].under_voltage = true;
        }
        limits_ok = false;
    } else {
        vmon_channels[curr_channel].over_voltage = false;
        vmon_channels[curr_channel].under_voltage = false;
    }

    voltage_string += String(vmon_channels[curr_channel].last_voltage) + ",";

    if (++curr_channel == NUM_VMON_CHANNELS) {
        curr_channel = 0;
        if (millis() > last_volt_log + VOLT_LOG_PERIOD) {
            storageManager.LogSD(voltage_string, VMON_DATA);
            last_volt_log = millis();
        }
        voltage_string = "";
    }

    return limits_ok;
}

bool MonitorMCB::CheckCurrents(void)
{
    static String current_string = "";
    static uint8_t curr_channel = 0;
    static uint32_t last_curr_log = 0;
    bool limits_ok = true;
    float raw = 0.0f;

    // read channel
    raw = analogRead(imon_channels[curr_channel].channel_pin);

    // calculate load current from sense current pin voltage (very sensitive to constants)
    imon_channels[curr_channel].last_current = SENSE_CURR_SLOPE *
            ((VREF / imon_channels[curr_channel].pulldown_res) * (raw / MAX_ADC_READ) - I_OFFSET);

    // check limits
    if (imon_channels[curr_channel].last_current > imon_channels[curr_channel].limit_hi) {
        if (!imon_channels[curr_channel].over_current) { // if newly over
            storageManager.LogSD("Over current", ERR_DATA);
            imon_channels[curr_channel].over_current = true;
            imon_channels[curr_channel].under_current = false;
        }
        limits_ok = false;
    } else if (imon_channels[curr_channel].last_current < imon_channels[curr_channel].limit_lo) {
        if (!imon_channels[curr_channel].under_current) { // if newly under
            storageManager.LogSD("Under current", ERR_DATA);
            imon_channels[curr_channel].over_current = false;
            imon_channels[curr_channel].under_current = true;
        }
        limits_ok = false;
    } else {
        imon_channels[curr_channel].over_current = false;
        imon_channels[curr_channel].under_current = false;
    }

    current_string += String(imon_channels[curr_channel].last_current) + ",";

    if (++curr_channel == NUM_IMON_CHANNELS) {
        curr_channel = 0;
        if (millis() > last_curr_log + CURR_LOG_PERIOD) {
            storageManager.LogSD(current_string, IMON_DATA);
            last_curr_log = millis();
        }
        current_string = "";
    }

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