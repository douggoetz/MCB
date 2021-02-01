/*
 *  Monitor.cpp
 *  File implementing the MCB monitor
 *  Author: Alex St. Clair
 *  October 2018
 */

#include "MonitorMCB.h"
#include "Serialize.h"

MonitorMCB::MonitorMCB(SafeBuffer * monitor_q, SafeBuffer * action_q, Reel * reel_in, LevelWind * lw_in, InternalSerialDriverMCB * dibdriver, ConfigManagerMCB * cfgManager)
    : storageManager()
    , tempC_MTR1(MTR1_THERM_CH)
    , tempC_MTR2(MTR2_THERM_CH)
    , tempC_MC1(MC1_THERM_CH)
    , tempC_SPARE1(SPARE1_THERM_CH)
    , tempC_SPARE2(SPARE2_THERM_CH)
    
    //: ltcManager(LTC_TEMP_CS_PIN, LTC_TEMP_RESET_PIN, THERM_SENSE_CH, RTD_SENSE_CH)
    //, storageManager()
{
    monitor_low_power = false;
    monitor_reel = false;
    monitor_levelwind = false;
    monitor_queue = monitor_q;
    action_queue = action_q;
    reel = reel_in;
    levelWind = lw_in;
    dibDriver = dibdriver;
    configManager = cfgManager;
}

void MonitorMCB::InitializeSensors(void)
{
	//analogReadRes(12);
	//for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
    //    ltcManager.channel_assignments[temp_sensors[i].channel_number] = temp_sensors[i].sensor_type;
   //     temp_sensors[i].sensor_error = true; // true until proven otherwise (affects TM averaging)
    //}
	//ltcManager.InitializeAndConfigure();

    if(!tempC_MC1.ValidateAddrCRC()){ //use to check that a valid sensor address is being retreived. 
        DEBUG_SERIAL.println("sensor crc bad, check sensor connection");
    }

    if(!tempC_MTR1.ValidateAddrCRC()){ //use to check that a valid sensor address is being retreived. 
        DEBUG_SERIAL.println("sensor crc bad, check sensor connection");
    }

    if(!tempC_MTR2.ValidateAddrCRC()){ //use to check that a valid sensor address is being retreived. 
        DEBUG_SERIAL.println("sensor crc bad, check sensor connection");
    }

    if(!tempC_SPARE1.ValidateAddrCRC()){ //use to check that a valid sensor address is being retreived. 
        DEBUG_SERIAL.println("sensor crc bad, check sensor connection");
    }

    if(!tempC_SPARE2.ValidateAddrCRC()){ //use to check that a valid sensor address is being retreived. 
        DEBUG_SERIAL.println("sensor crc bad, check sensor connection");
    }

    tempC_MTR1.ManageState(T_MTR1);
    tempC_MTR2.ManageState(T_MTR2);
    tempC_MC1.ManageState(T_MC1);
    tempC_SPARE1.ManageState(T_SP1);
    tempC_SPARE2.ManageState(T_SP2); 

}

void MonitorMCB::UpdateLimits(void)
{
    // temp sensor limits
    temp_sensors[MTR1_THERM].limit_hi = configManager->mtr1_temp_lim.Read().hi;
    temp_sensors[MTR1_THERM].limit_lo = configManager->mtr1_temp_lim.Read().lo;
    temp_sensors[MTR2_THERM].limit_hi = configManager->mtr2_temp_lim.Read().hi;
    temp_sensors[MTR2_THERM].limit_lo = configManager->mtr2_temp_lim.Read().lo;
    temp_sensors[MC1_THERM].limit_hi = configManager->mc1_temp_lim.Read().hi;
    temp_sensors[MC1_THERM].limit_lo = configManager->mc1_temp_lim.Read().lo;
    temp_sensors[SPARE1_THERM].limit_hi = configManager->mc2_temp_lim.Read().hi;
    temp_sensors[SPARE1_THERM].limit_lo = configManager->mc2_temp_lim.Read().lo;
    temp_sensors[SPARE2_THERM].limit_hi = configManager->dcdc_temp_lim.Read().hi;
    temp_sensors[SPARE2_THERM].limit_lo = configManager->dcdc_temp_lim.Read().lo;
    //temp_sensors[SPARE_THERM].limit_hi = configManager->spare_therm_lim.Read().hi;
    //temp_sensors[SPARE_THERM].limit_lo = configManager->spare_therm_lim.Read().lo;

    // vmon limits
    vmon_channels[VMON_3V3].limit_hi = configManager->vmon_3v3_lim.Read().hi;
    vmon_channels[VMON_3V3].limit_lo = configManager->vmon_3v3_lim.Read().lo;
    vmon_channels[VMON_15V].limit_hi = configManager->vmon_15v_lim.Read().hi;
    vmon_channels[VMON_15V].limit_lo = configManager->vmon_15v_lim.Read().lo;
    vmon_channels[VMON_20V].limit_hi = configManager->vmon_20v_lim.Read().hi;
    vmon_channels[VMON_20V].limit_lo = configManager->vmon_20v_lim.Read().lo;
    vmon_channels[VMON_SPOOL].limit_hi = configManager->vmon_spool_lim.Read().hi;
    vmon_channels[VMON_SPOOL].limit_lo = configManager->vmon_spool_lim.Read().lo;

    // imon limits
    imon_channels[IMON_BRK].limit_hi = configManager->imon_brake_lim.Read().hi;
    imon_channels[IMON_BRK].limit_lo = configManager->imon_brake_lim.Read().lo;
    imon_channels[IMON_MC].limit_hi = configManager->imon_mc_lim.Read().hi;
    imon_channels[IMON_MC].limit_lo = configManager->imon_mc_lim.Read().lo;
    imon_channels[IMON_MTR1].limit_hi = configManager->imon_mtr1_lim.Read().hi;
    imon_channels[IMON_MTR1].limit_lo = configManager->imon_mtr1_lim.Read().lo;
    imon_channels[IMON_MTR2].limit_hi = configManager->imon_mtr2_lim.Read().hi;
    imon_channels[IMON_MTR2].limit_lo = configManager->imon_mtr2_lim.Read().lo;
    //imon_channels[IMON_INST].limit_hi = configManager->imon_inst_lim.Read().hi;
    //imon_channels[IMON_INST].limit_lo = configManager->imon_inst_lim.Read().lo;

    // torque limits
    motor_torques[REEL_INDEX].limit_hi = configManager->reel_torque_lim.Read().hi;
    motor_torques[REEL_INDEX].limit_lo = configManager->reel_torque_lim.Read().lo;
    motor_torques[LEVEL_WIND_INDEX].limit_hi = configManager->lw_torque_lim.Read().hi;
    motor_torques[LEVEL_WIND_INDEX].limit_lo = configManager->lw_torque_lim.Read().lo;
}

void MonitorMCB::Monitor(void)
{
    bool limits_ok = true;

    HandleCommands();

    CheckVoltages(); // no active voltage limits
    limits_ok &= CheckCurrents();
    if (!monitor_low_power) {
        limits_ok &= CheckTemperatures();
    }

    if (monitor_reel || monitor_levelwind) {
        limits_ok &= CheckTorques();
        UpdatePositions();
        if (AggregateMotionData()) { // returns true if ready to send
            SendMotionData();
        }
    }

    if (!limits_ok) {
        action_queue->Push(ACT_LIMIT_EXCEEDED); // notify state manager
    }

    tempC_MTR1.ManageState(T_MTR1);
    tempC_MTR2.ManageState(T_MTR2);
    tempC_MC1.ManageState(T_MC1);
    tempC_SPARE1.ManageState(T_SP1);
    tempC_SPARE2.ManageState(T_SP2); 

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
                //ltcManager.WakeUp();
                monitor_low_power = false;
            }
            break;
        case MONITOR_REEL_ON:
            monitor_reel = true;
            monitor_levelwind = false;
            if (monitor_low_power) {
                //ltcManager.WakeUp();
                monitor_low_power = false;
            }
            break;
        case MONITOR_MOTORS_OFF:
            monitor_reel = false;
            monitor_levelwind = false;
            if (monitor_low_power) {
                //ltcManager.WakeUp();
                monitor_low_power = false;
            }
            break;
        case MONITOR_LOW_POWER:
            //ltcManager.Sleep();
            monitor_low_power = true;
            monitor_reel = false;
            monitor_levelwind = false;
            break;
        case MONITOR_SEND_TEMPS:
            //if (monitor_low_power) {
              //  dibDriver->dibComm.TX_Temperatures(LTC_POWERED_OFF, LTC_POWERED_OFF, LTC_POWERED_OFF, LTC_POWERED_OFF, LTC_POWERED_OFF, LTC_POWERED_OFF);
            //} else {
            dibDriver->dibComm.TX_Temperatures(temp_sensors[0].last_temperature, temp_sensors[1].last_temperature, temp_sensors[2].last_temperature,
                                                   temp_sensors[3].last_temperature, temp_sensors[4].last_temperature);
            //}
            Serial.print("Temps: ");
            Serial.print(temp_sensors[0].last_temperature); Serial.print(",");
            Serial.print(temp_sensors[1].last_temperature); Serial.print(",");
            Serial.print(temp_sensors[2].last_temperature); Serial.print(",");
            Serial.print(temp_sensors[3].last_temperature); Serial.print(",");
            Serial.println(temp_sensors[4].last_temperature);
            break;
        case MONITOR_SEND_VOLTS:
            dibDriver->dibComm.TX_Voltages(vmon_channels[0].last_voltage, vmon_channels[1].last_voltage, vmon_channels[2].last_voltage, vmon_channels[3].last_voltage);
            Serial.print("Voltages: ");
            Serial.print(vmon_channels[0].last_voltage); Serial.print(",");
            Serial.print(vmon_channels[1].last_voltage); Serial.print(",");
            Serial.print(vmon_channels[2].last_voltage); Serial.print(",");
            Serial.println(vmon_channels[3].last_voltage);
            break;
            
        case MONITOR_SEND_CURRS:
            dibDriver->dibComm.TX_Currents(imon_channels[0].last_current, imon_channels[1].last_current, imon_channels[2].last_current, imon_channels[3].last_current, imon_channels[4].last_current);
            Serial.print("Currents: ");
            Serial.print(imon_channels[0].last_current); Serial.print(",");
            Serial.print(imon_channels[1].last_current); Serial.print(",");
            Serial.print(imon_channels[2].last_current); Serial.print(",");
            Serial.print(imon_channels[3].last_current); Serial.print(",");
            Serial.println(imon_channels[4].last_current);
            break;
        case UNUSED_COMMAND:
        default:
            storageManager.LogSD("Unknown monitor queue error", ERR_DATA);
            break;
        }
    }
}

bool MonitorMCB::VerifyDeployVoltage(void)
{
    return vmon_channels[VMON_15V].last_voltage > MIN_DEPLOY_VOLTAGE;
}

// measures one sensor at a time in a non-blocking fashion
bool MonitorMCB::CheckTemperatures(void)
{
    static String temperature_string = "";
    static uint8_t curr_sensor = 0;
    //static bool measurement_ongoing = false;
    static uint32_t last_temp_log = 0;
    bool limits_ok = true;
    float temp = 0.0f;

    //if (!measurement_ongoing) {
    //    ltcManager.StartMeasurement(temp_sensors[curr_sensor].channel_number);
    //    measurement_ongoing = true;
    //} else if (ltcManager.FinishedMeasurement()) {
      //  measurement_ongoing = false;
    // temp = ltcManager.ReadMeasurementResult(temp_sensors[curr_sensor].channel_number);

     for(curr_sensor = 0; curr_sensor < NUM_TEMP_SENSORS; curr_sensor++){

        switch (curr_sensor)
        {
            case MTR1_THERM:
                temp = T_MTR1;
                break;
            case MTR2_THERM:
                temp = T_MTR2;
                break;
            case MC1_THERM:
                temp = T_MC1;
                break;
            case SPARE1_THERM:
                temp = T_SP1;
                break;
            case SPARE2_THERM:
                temp = T_SP2;
                break;
            default:
                temp = -999;
                break;
        }

        temp_sensors[curr_sensor].last_temperature = temp;
        temp_sensors[curr_sensor].sensor_error = false;

        if (temp > temp_sensors[curr_sensor].limit_hi) {
                if (!temp_sensors[curr_sensor].over_temp) { // if newly over
                    storageManager.LogSD("Over temperature", ERR_DATA);
                    temp_sensors[curr_sensor].over_temp = true;
                    temp_sensors[curr_sensor].under_temp = false;
                }
                snprintf(limit_error, 100, "MCB Temp Limit, ch %u, t=%f", curr_sensor, temp);
                limits_ok = false;
            } else if (temp < temp_sensors[curr_sensor].limit_lo) {
                if (!temp_sensors[curr_sensor].under_temp) { // if newly under
                    storageManager.LogSD("Under temperature", ERR_DATA);
                    temp_sensors[curr_sensor].over_temp = false;
                    temp_sensors[curr_sensor].under_temp = true;
                }
                snprintf(limit_error, 100, "MCB Temp Limit, ch %u, t=%f", curr_sensor, temp);
                limits_ok = false;
            } else {
                temp_sensors[curr_sensor].over_temp = false;
                temp_sensors[curr_sensor].under_temp = false;
            }


        temperature_string += String(temp) + ",";

     }

        curr_sensor = 0;
        if (millis() > last_temp_log + TEMP_LOG_PERIOD) {
            storageManager.LogSD(temperature_string, TEMP_DATA);
            last_temp_log = millis();
        }
        temperature_string = "";

    //     // validate reading, check limits if valid
    //     if (temp != TEMPERATURE_ERROR && temp != LTC_SENSOR_ERROR) {
    //         temp_sensors[curr_sensor].last_temperature = temp;
    //         temp_sensors[curr_sensor].sensor_error = false;

    //         if (temp > temp_sensors[curr_sensor].limit_hi) {
    //             if (!temp_sensors[curr_sensor].over_temp) { // if newly over
    //                 storageManager.LogSD("Over temperature", ERR_DATA);
    //                 temp_sensors[curr_sensor].over_temp = true;
    //                 temp_sensors[curr_sensor].under_temp = false;
    //             }
    //             snprintf(limit_error, 100, "MCB Temp Limit, ch %u, t=%f", curr_sensor, temp);
    //             limits_ok = false;
    //         } else if (temp < temp_sensors[curr_sensor].limit_lo) {
    //             if (!temp_sensors[curr_sensor].under_temp) { // if newly under
    //                 storageManager.LogSD("Under temperature", ERR_DATA);
    //                 temp_sensors[curr_sensor].over_temp = false;
    //                 temp_sensors[curr_sensor].under_temp = true;
    //             }
    //             snprintf(limit_error, 100, "MCB Temp Limit, ch %u, t=%f", curr_sensor, temp);
    //             limits_ok = false;
    //         } else {
    //             temp_sensors[curr_sensor].over_temp = false;
    //             temp_sensors[curr_sensor].under_temp = false;
    //         }
    //     } else {
    //         temp_sensors[curr_sensor].sensor_error = true;
    //     }

    //     temperature_string += String(temp) + ",";

    //     if (++curr_sensor == NUM_TEMP_SENSORS) {
    //         curr_sensor = 0;
    //         if (millis() > last_temp_log + TEMP_LOG_PERIOD) {
    //             storageManager.LogSD(temperature_string, TEMP_DATA);
    //             last_temp_log = millis();
    //         }
    //         temperature_string = "";
    //     }
    // }

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
    vmon_channels[curr_channel].last_raw = analogRead(vmon_channels[curr_channel].channel_pin);
    raw = vmon_channels[curr_channel].last_raw;

    // calculate voltage given the resistor divider network
    vmon_channels[curr_channel].last_voltage = VREF * (raw / MAX_ADC_READ) / vmon_channels[curr_channel].voltage_divider;

    // check limits
    if (vmon_channels[curr_channel].last_voltage > vmon_channels[curr_channel].limit_hi) {
        if (!vmon_channels[curr_channel].over_voltage) { // if newly over
            storageManager.LogSD("Over voltage", ERR_DATA);
            vmon_channels[curr_channel].over_voltage = true;
            vmon_channels[curr_channel].under_voltage = false;
        }
        snprintf(limit_error, 100, "MCB Volt Limit, ch %u, V=%f", curr_channel, vmon_channels[curr_channel].last_voltage);
        limits_ok = false;
    } else if (vmon_channels[curr_channel].last_voltage < vmon_channels[curr_channel].limit_lo) {
        if (!vmon_channels[curr_channel].under_voltage) { // if newly under
            storageManager.LogSD("Under voltage", ERR_DATA);
            vmon_channels[curr_channel].over_voltage = false;
            vmon_channels[curr_channel].under_voltage = true;
        }
        snprintf(limit_error, 100, "MCB Volt Limit, ch %u, V=%f", curr_channel, vmon_channels[curr_channel].last_voltage);
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

    float coadd = 0.0f;

    // read channel // 

    for (int i = 0; i < 100; i++){ // find average over 100 reads 

        //delay(10);
        coadd+=analogRead(imon_channels[curr_channel].channel_pin);
        
        //if(i = 99){
          //  imon_channels[curr_channel].last_raw = coadd/100;
        //}       
    }

    imon_channels[curr_channel].last_raw = coadd/100.0;
    raw = imon_channels[curr_channel].last_raw;
    coadd = 0.0;

    // calculate load current from sense current pin voltage (very sensitive to constants)
    if(curr_channel == IMON_INST){

        imon_channels[curr_channel].last_current = ((1.65-(VREF * raw / MAX_ADC_READ))/.044); //44 mv/A and minimum V=1.65 based on ACS&1240LLCBTR-030B3 datasheet
    }
    else{

        imon_channels[curr_channel].last_current = SENSE_CURR_SLOPE *
            ((VREF / imon_channels[curr_channel].pulldown_res) * (raw / MAX_ADC_READ) - I_OFFSET);
    
    }

    // check limits
    if (imon_channels[curr_channel].last_current > imon_channels[curr_channel].limit_hi) {
        if (!imon_channels[curr_channel].over_current) { // if newly over
            storageManager.LogSD("Over current", ERR_DATA);
            Serial.print("OC channel: "); Serial.println(curr_channel);
            Serial.println(imon_channels[curr_channel].last_current);
            imon_channels[curr_channel].over_current = true;
            imon_channels[curr_channel].under_current = false;
        }
        snprintf(limit_error, 100, "MCB Curr Limit, ch %u, I=%f", curr_channel, imon_channels[curr_channel].last_current);
        limits_ok = false;
    } else if (imon_channels[curr_channel].last_current < imon_channels[curr_channel].limit_lo) {
        if (!imon_channels[curr_channel].under_current) { // if newly under
            storageManager.LogSD("Under current", ERR_DATA);
            imon_channels[curr_channel].over_current = false;
            imon_channels[curr_channel].under_current = true;
        }
        snprintf(limit_error, 100, "MCB Curr Limit, ch %u, I=%f", curr_channel, imon_channels[curr_channel].last_current);
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
                snprintf(limit_error, 100, "MCB Reel Torque Limit, t=%f", motor_torques[REEL_INDEX].last_torque);
                limits_ok = false;
            } else if (motor_torques[REEL_INDEX].last_torque < motor_torques[REEL_INDEX].limit_lo) {
                if (!motor_torques[REEL_INDEX].under_torque) { // if newly under
                    storageManager.LogSD("Under torque, reel", ERR_DATA);
                    motor_torques[REEL_INDEX].over_torque = false;
                    motor_torques[REEL_INDEX].under_torque = true;
                }
                snprintf(limit_error, 100, "MCB Reel Torque Limit, t=%f", motor_torques[REEL_INDEX].last_torque);
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
                snprintf(limit_error, 100, "MCB LW Torque Limit, t=%f", motor_torques[LEVEL_WIND_INDEX].last_torque);
                limits_ok = false;
            } else if (motor_torques[LEVEL_WIND_INDEX].last_torque < motor_torques[LEVEL_WIND_INDEX].limit_lo) {
                if (!motor_torques[LEVEL_WIND_INDEX].under_torque) { // if newly under
                    storageManager.LogSD("Under torque, level wind", ERR_DATA);
                    motor_torques[LEVEL_WIND_INDEX].over_torque = false;
                    motor_torques[LEVEL_WIND_INDEX].under_torque = true;
                }
                snprintf(limit_error, 100, "MCB LW Torque Limit, t=%f", motor_torques[LEVEL_WIND_INDEX].last_torque);
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

// called at about 1 Hz during motion, return true if data ready to send
bool MonitorMCB::AggregateMotionData(void)
{
    // add the torques read from the MCs as long as they were valid
    if (!motor_torques[REEL_INDEX].read_error) {
        AddFastTM(&reel_torques, TorqueToUInt16(motor_torques[REEL_INDEX].last_torque));
    }
    if (!motor_torques[LEVEL_WIND_INDEX].read_error) {
        AddFastTM(&lw_torques, TorqueToUInt16(motor_torques[LEVEL_WIND_INDEX].last_torque));
    }

    // add the motor currents (ADC read can't fail in software)
    AddFastTM(&reel_currents, imon_channels[IMON_MTR1].last_raw);
    AddFastTM(&lw_currents, imon_channels[IMON_MTR2].last_raw);

    // add the slow param temperatures as long as they were valid
    if (!temp_sensors[MTR1_THERM].sensor_error) {
        AddSlowTM(PARAM_REEL_TEMP, TempToUInt16(temp_sensors[MTR1_THERM].last_temperature));
    }
    if (!temp_sensors[MTR2_THERM].sensor_error) {
        AddSlowTM(PARAM_LW_TEMP, TempToUInt16(temp_sensors[MTR2_THERM].last_temperature));
    }
    if (!temp_sensors[MC1_THERM].sensor_error) {
        AddSlowTM(PARAM_MC1_TEMP, TempToUInt16(temp_sensors[MC1_THERM].last_temperature));
    }
    if (!temp_sensors[SPARE1_THERM].sensor_error) {
        AddSlowTM(PARAM_MC2_TEMP, TempToUInt16(temp_sensors[SPARE1_THERM].last_temperature));
    }

    // add the brake current and supply voltage (ADC read can't fail in software)
    AddSlowTM(PARAM_BRAKE_CURR, imon_channels[IMON_BRK].last_raw);
    AddSlowTM(PARAM_SUPPLY_VOLT, vmon_channels[VMON_15V].last_raw);

    // check if ready to send
    if (millis() > last_send_millis + 9500) {
        last_send_millis = millis();
        return true;
    } else {
        return false;
    }
}

void MonitorMCB::SendMotionData(void)
{
    uint16_t buffer_index = 0;
    bool buffer_success = true;

    // add the rotating parameter ID
    buffer_success &= BufferAddUInt8(rotating_parameter, tm_buffer, MOTION_TM_SIZE, &buffer_index);

    // add the rotating parameter values
    buffer_success &= BufferAddUInt16(AverageResetSlowTM((RotatingParam_t) rotating_parameter), tm_buffer, MOTION_TM_SIZE, &buffer_index);
    buffer_success &= BufferAddUInt16(slow_tm[rotating_parameter].running_max, tm_buffer, MOTION_TM_SIZE, &buffer_index);

    // reset the rotating_parameter maximum and switch to the next parameter
    slow_tm[rotating_parameter].running_max = 0;
    rotating_parameter = (NUM_ROTATING_PARAMS == rotating_parameter + 1) ? FIRST_ROTATING_PARAM : rotating_parameter + 1;

    // add the reel and level wind torques and currents
    buffer_success &= BufferAddUInt16(AverageResetFastTM(&reel_torques), tm_buffer, MOTION_TM_SIZE, &buffer_index);
    buffer_success &= BufferAddUInt16(reel_torques.running_max, tm_buffer, MOTION_TM_SIZE, &buffer_index);
    buffer_success &= BufferAddUInt16(AverageResetFastTM(&lw_torques), tm_buffer, MOTION_TM_SIZE, &buffer_index);
    buffer_success &= BufferAddUInt16(lw_torques.running_max, tm_buffer, MOTION_TM_SIZE, &buffer_index);
    buffer_success &= BufferAddUInt16(AverageResetFastTM(&reel_currents), tm_buffer, MOTION_TM_SIZE, &buffer_index);
    buffer_success &= BufferAddUInt16(reel_currents.running_max, tm_buffer, MOTION_TM_SIZE, &buffer_index);
    buffer_success &= BufferAddUInt16(AverageResetFastTM(&lw_currents), tm_buffer, MOTION_TM_SIZE, &buffer_index);
    buffer_success &= BufferAddUInt16(lw_currents.running_max, tm_buffer, MOTION_TM_SIZE, &buffer_index);

    // reset the reel and level wind torque and current maxes as well as reel speed
    reel_torques.running_max = 0;
    lw_torques.running_max = 0;
    reel_currents.running_max = 0;
    lw_currents.running_max = 0;

    // add the reel and lw positions
    buffer_success &= BufferAddFloat(reel->absolute_position / REEL_UNITS_PER_REV, tm_buffer, MOTION_TM_SIZE, &buffer_index);
    buffer_success &= BufferAddFloat(levelWind->absolute_position, tm_buffer, MOTION_TM_SIZE, &buffer_index);

    // check for a good buffer and send it off
    if (buffer_success && MOTION_TM_SIZE == buffer_index) {
        dibDriver->dibComm.AssignBinaryTXBuffer(tm_buffer, MOTION_TM_SIZE, MOTION_TM_SIZE);
        dibDriver->dibComm.TX_Bin(MCB_MOTION_TM);
        Serial.println("Sent motion TM packet");
    } else {
        storageManager.LogSD("Error creating TM buffer", ERR_DATA);
    }
}

void MonitorMCB::AddFastTM(Fast_TM_t * tm_type, uint16_t data)
{
    // check the data vs the running maximum
    if (data > tm_type->running_max) tm_type->running_max = data;

    // make sure the averaging array isn't full
    if (FAST_TM_MAX_SAMPLES == tm_type->curr_index) return;

    // add the data to the array (index incremented externally for all fast TM)
    tm_type->data[tm_type->curr_index++] = data;
}

void MonitorMCB::AddSlowTM(RotatingParam_t tm_index, uint16_t data)
{
    // check the data vs the running maximum
    if (data > slow_tm[tm_index].running_max) slow_tm[tm_index].running_max = data;

    // make sure the averaging array isn't already full
    if (SLOW_TM_MAX_SAMPLES == slow_tm[tm_index].curr_index) return;

    // add the data to the array and increment the index
    slow_tm[tm_index].data[slow_tm[tm_index].curr_index++] = data;
}

uint16_t MonitorMCB::AverageResetFastTM(Fast_TM_t * tm_type)
{
    float result = 0;

    // if no data points have been collected, return 0
    if (0 == tm_type->curr_index) return 0;

    // calculate the average
    for (uint8_t i = 0; i < tm_type->curr_index; i++) {
        result += tm_type->data[i];
    }
    result = result / (float) tm_type->curr_index;

    // reset the data array
    tm_type->curr_index = 0;

    return (uint16_t) result;
}

uint16_t MonitorMCB::AverageResetSlowTM(RotatingParam_t tm_index)
{
    float result = 0;

    // if no data points have been collected, return 0
    if (0 == slow_tm[tm_index].curr_index) return 0;

    // calculate the average
    for (uint8_t i = 0; i < slow_tm[tm_index].curr_index; i++) {
        result += slow_tm[tm_index].data[i];
    }
    result = result / (float) slow_tm[tm_index].curr_index;

    // reset the data array
    slow_tm[tm_index].curr_index = 0;

    return (uint16_t) result;
}

// turn float to uint16_t with resolution 0.1 N
uint16_t MonitorMCB::TorqueToUInt16(float torque)
{
    torque *= 10.0f; // move the decimal to the right by one to capture tenths
    torque += 30000; // add an offset to deal with negative torques

    // ensure we're within the range of a uint16_t
    if (UINT16_MAX <= torque) return UINT16_MAX;
    if (0 >= torque) return 0;

    return (uint16_t) torque;
}

// turn float into uint16_t with resolution 0.1 C
uint16_t MonitorMCB::TempToUInt16(float temp)
{
    temp *= 10.0f; // move the decimal to the right by one to capture tenths
    temp += 30000; // add an offset to deal with negative temperatures

    // ensure we're within the range of a uint16_t
    if (UINT16_MAX <= temp) return UINT16_MAX;
    if (0 >= temp) return 0;

    return (uint16_t) temp;
}