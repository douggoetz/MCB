/*
 *  StateManagerMCB.h
 *  File defining the MCB monitor
 *  Author: Alex St. Clair
 *  October 2018
 */

#ifndef MONITORMCB_H_
#define MONITORMCB_H_

#include "StorageManagerMCB.h"
#include "LTC2983Manager.h"
#include "ActionsMCB.h"
#include "LevelWind.h"
#include "Reel.h"
#include "Queue.h"
#include <stdint.h>

#define TEMP_PERIOD     5000 // min period for temperature measurements in millis
#define ADC_PERIOD      500  // min period for voltage/current measurements in millis

enum Monitor_Command_t : uint8_t {
    MONITOR_BOTH_MOTORS_ON,
    MONITOR_REEL_ON,
    MONITOR_MOTORS_OFF,
    MONITOR_LOW_POWER,
    UNUSED_COMMAND
};

enum Motor_Torque_Index_t : uint8_t {
    REEL_INDEX = 0,
    LEVEL_WIND_INDEX = 1
};

struct Temp_Sensor_t {
    float last_temperature;
    float limit_hi;
    float limit_lo;
    uint8_t channel_number;
    Sensor_Type_t sensor_type;
    bool sensor_error;
    bool over_temp;
    bool under_temp;
};

struct ADC_Voltage_t {
    float last_voltage;
    float limit_hi;
    float limit_lo;
    float voltage_divider;
    uint8_t channel_pin;
    bool over_voltage;
    bool under_voltage;
};

struct ADC_Current_t {
    float last_current;
    float limit_hi;
    float limit_lo;
    float pulldown_res;
    uint8_t channel_pin;
    bool over_current;
    bool under_current;
};

struct Motor_Torque_t {
    float last_torque;
    float limit_hi;
    float limit_lo;
    float conversion;
    bool read_error;
    bool over_torque;
    bool under_torque;
};

class MonitorMCB {
public:
    MonitorMCB(Queue * monitor_q, Queue * action_q, Reel * reel_in, LevelWind * lw_in);
    ~MonitorMCB(void) { };
    
    // interface methods
    void InitializeSensors(void);
    void UpdateLimits(void);
    void Monitor(void);
private:
    // helper methods
    void HandleCommands(void);
    bool CheckTemperatures(void);
    bool CheckVoltages(void);
    bool CheckCurrents(void);
    bool CheckTorques(void);
    void UpdatePositions(void);
    void PrintMotorData(void);

    // state variables
    bool monitor_reel;
    bool monitor_levelwind;
    bool monitor_low_power;

    // communication with state manager
    Queue * monitor_queue;
    Queue * action_queue;

    // hardware objects
	LTC2983Manager ltcManager;
	StorageManagerMCB storageManager;
    LevelWind * levelWind;
    Reel * reel;

    // temperature sensor table (hard-coded limits will be replace at init from EEPROM)
    Temp_Sensor_t temp_sensors[NUM_TEMP_SENSORS] = 
        /* last_temp | limit_hi | limit_lo | channel_num    | channel_type     | sens_err | over_temp | under_temp */
        {{ 0.0f,       100.0f,     -100.0f,  MTR1_THERM_CH,   THERMISTOR_44006,  false,     false,      false},
         { 0.0f,       100.0f,     -100.0f,  MTR2_THERM_CH,   THERMISTOR_44006,  false,     false,      false},
         { 0.0f,       100.0f,     -100.0f,  MC1_THERM_CH,    THERMISTOR_44006,  false,     false,      false},
         { 0.0f,       100.0f,     -100.0f,  MC2_THERM_CH,    THERMISTOR_44006,  false,     false,      false},
         { 0.0f,       100.0f,     -100.0f,  DCDC_THERM_CH,   THERMISTOR_44006,  false,     false,      false},
         { 0.0f,       100.0f,     -100.0f,  SPARE_THERM_CH,  THERMISTOR_44006,  false,     false,      false}};

    // voltage ADC channel table (hard-coded limits will be replaced at init from EEPROM)
    ADC_Voltage_t vmon_channels[NUM_VMON_CHANNELS] =
        /* last_volt | limit_hi | limit_lo | volt_div | channel_pin   | over_volt | under_volt */
        {{ 0.0f,       3.8f,      2.6f,      0.5f,      A_VMON_3V3,     false,      false},
         { 0.0f,       20.0f,     12.0f,     0.102f,    A_VMON_15V,     false,      false},
         { 0.0f,       26.0f,     18.0f,     0.0746f,   A_VMON_20V,     false,      false},
         { 0.0f,       3.6f,      0.0f,      1,         A_SPOOL_LEVEL,  false,      false}};
    
    // current ADC channel table (hard-coded limits will be replaced at init from EEPROM)
    ADC_Current_t imon_channels[NUM_IMON_CHANNELS] = 
        /* last_curr | limit_hi | limit_lo | pulldown_res | channel_pin  | over_curr | under_curr */
        {{ 0.0f,       5.0f,      -2.0f,     RES_15K,       A_IMON_BRK,    false,      false},
         { 0.0f,       5.0f,      -2.0f,     RES_15K,       A_IMON_MC,     false,      false},
         { 0.0f,       4.5f,      -2.0f,     RES_2K,        A_IMON_MTR1,   false,      false},
         { 0.0f,       5.0f,      -2.0f,     RES_2K,        A_IMON_MTR2,   false,      false}};

    Motor_Torque_t motor_torques[2] = 
        /* last torque | limit_hi | limit_lo | conversion | read_error | over_torque | under_torque */
        {{0.0f,          500.0f,    -500.0f,   500.0f,      false,       false,        false},
         {0.0f,          500.0f,    -500.0f,   1.0f,        false,       false,        false}};
};

#endif