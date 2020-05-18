/*
 *  StateManagerMCB.h
 *  File defining the MCB monitor
 *  Author: Alex St. Clair
 *  October 2018
 */

#ifndef MONITORMCB_H_
#define MONITORMCB_H_

#include "InternalSerialDriverMCB.h"
#include "StorageManagerMCB.h"
#include "ConfigManagerMCB.h"
#include "LTC2983Manager.h"
#include "ActionsMCB.h"
#include "LevelWind.h"
#include "Reel.h"
#include "SafeBuffer.h"
#include <stdint.h>

#define TEMP_LOG_PERIOD     30000  // milliseconds
#define VOLT_LOG_PERIOD     30000  // milliseconds
#define CURR_LOG_PERIOD     30000  // milliseconds

#define FAST_TM_MAX_SAMPLES 10
#define SLOW_TM_MAX_SAMPLES (NUM_ROTATING_PARAMS * FAST_TM_MAX_SAMPLES)

#define MIN_DEPLOY_VOLTAGE  14.0f

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
    uint16_t last_raw;
    uint8_t channel_pin;
    bool over_voltage;
    bool under_voltage;
};

struct ADC_Current_t {
    float last_current;
    float limit_hi;
    float limit_lo;
    float pulldown_res;
    uint16_t last_raw;
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

struct Slow_TM_t {
    uint16_t data[SLOW_TM_MAX_SAMPLES];
    uint16_t running_max;
    uint8_t curr_index;
};

struct Fast_TM_t {
    uint16_t data[FAST_TM_MAX_SAMPLES];
    uint16_t running_max;
    uint8_t curr_index;
};

class MonitorMCB {
public:
    MonitorMCB(SafeBuffer * monitor_q, SafeBuffer * action_q, Reel * reel_in, LevelWind * lw_in, InternalSerialDriverMCB * dibdriver, ConfigManagerMCB * cfgManager);
    ~MonitorMCB(void) { };

    // interface methods
    void InitializeSensors(void);
    void UpdateLimits(void);
    void Monitor(void);

    bool VerifyDeployVoltage(void);

    // if a limit is exceeded, the relevant info is written to this string
    char limit_error[100];// temperature sensor table (hard-coded limits will be replace at init from EEPROM)

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
        /* last_volt | limit_hi | limit_lo | volt_div | last_raw | channel_pin   | over_volt | under_volt */
        {{ 0.0f,       3.8f,      2.6f,      0.5f,      0,         A_VMON_3V3,     false,      false},
         { 0.0f,       20.0f,     12.0f,     0.102f,    0,         A_VMON_15V,     false,      false},
         { 0.0f,       26.0f,     18.0f,     0.0746f,   0,         A_VMON_20V,     false,      false},
         { 0.0f,       3.6f,      0.0f,      1,         0,         A_SPOOL_LEVEL,  false,      false}};

    // current ADC channel table (hard-coded limits will be replaced at init from EEPROM)
    ADC_Current_t imon_channels[NUM_IMON_CHANNELS] =
        /* last_curr | limit_hi | limit_lo | pulldown_res | last_raw | channel_pin  | over_curr | under_curr */
        {{ 0.0f,       5.0f,      -2.0f,     RES_15K,       0,         A_IMON_BRK,    false,      false},
         { 0.0f,       5.0f,      -2.0f,     RES_15K,       0,         A_IMON_MC,     false,      false},
         { 0.0f,       4.5f,      -2.0f,     RES_2K,        0,         A_IMON_MTR1,   false,      false},
         { 0.0f,       5.0f,      -2.0f,     RES_2K,        0,         A_IMON_MTR2,   false,      false}};

    Motor_Torque_t motor_torques[2] =
        /* last torque | limit_hi | limit_lo | conversion | read_error | over_torque | under_torque */
        {{0.0f,          500.0f,    -500.0f,   500.0f,      false,       false,        false},
         {0.0f,          500.0f,    -500.0f,   1.0f,        false,       false,        false}};


private:
    // read and respond to commands on the monitor_queue
    void HandleCommands(void);

    // functions for gathering and checking all of the sensor data
    bool CheckTemperatures(void);
    bool CheckVoltages(void);
    bool CheckCurrents(void);
    bool CheckTorques(void);
    void UpdatePositions(void);

    // old function for printing motor data
    void PrintMotorData(void);

    // called from the Monitor loop during motions
    bool AggregateMotionData(void);
    void SendMotionData(void);

    // helpers for interfacing with TM averagers
    void AddFastTM(Fast_TM_t * tm_type, uint16_t data);
    void AddSlowTM(RotatingParam_t tm_index, uint16_t data);
    uint16_t AverageResetFastTM(Fast_TM_t * tm_type);
    uint16_t AverageResetSlowTM(RotatingParam_t tm_index);

    // conversion helpers
    uint16_t TorqueToUInt16(float torque);
    uint16_t TempToUInt16(float temp);

    // state variables
    bool monitor_reel;
    bool monitor_levelwind;
    bool monitor_low_power;

    // communication with state manager
    SafeBuffer * monitor_queue;
    SafeBuffer * action_queue;

    // hardware objects
	LTC2983Manager ltcManager;
	StorageManagerMCB storageManager;
    LevelWind * levelWind;
    Reel * reel;
    InternalSerialDriverMCB * dibDriver;
    ConfigManagerMCB * configManager;

    // motion data buffer for sending to the DIB/PIB
    uint8_t tm_buffer[MOTION_TM_SIZE];

    // motion data trackers
    uint32_t last_send_millis = 0;
    uint8_t rotating_parameter = FIRST_ROTATING_PARAM; // index  = 0

    // running averages for motion telemetry (fast - 10s period)
    Fast_TM_t reel_torques = {{0},0,0};
    Fast_TM_t lw_torques = {{0},0,0};
    Fast_TM_t reel_currents = {{0},0,0};
    Fast_TM_t lw_currents = {{0},0,0};

    Slow_TM_t slow_tm[NUM_ROTATING_PARAMS] = {{{0},0,0}};

};

#endif