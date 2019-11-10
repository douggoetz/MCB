/*
 *  StorageManagerMCB.cpp
 *  File implementing the class that manages SD and EEPROM storage
 *  Author: Alex St. Clair
 *  March 2018
 *
 *  Note: this class is implemented so that it can be used across multiple
 *        instances. This means that all data members MUST be static.
 */

#include "StorageManagerMCB.h"
#include "Reel.h"

bool StorageManagerMCB::sd_state = false;
EEPROM_Data_t StorageManagerMCB::eeprom_data = {0};
String StorageManagerMCB::base_directory = "";

Log_Data_Info_t StorageManagerMCB::log_data_info[NUM_LOG_DATA] = {
	// num_writes | max_writes | num_files | file_prefix
	{  0          , 100        , 0         , "VMON_"     }, // VMON_DATA
	{  0          , 100        , 0         , "IMON_"     }, // IMON_DATA
	{  0          , 100        , 0         , "TEMP_"     }, // TEMP_DATA
	{  0          , 20         , 0         , "ERR_"      }, // ERR_DATA
	{  0          , 100        , 0         , "MOT_"      }  // MOTION_DATA
};

// TODO: add timestamp to writes

// Methods for SD logging -----------------------------------------------------
bool StorageManagerMCB::LogSD(uint8_t * log_data, int data_size, Log_Data_Type_t data_type) {
	if (!sd_state) {
		return false;
	}

	// TODO: implement binary data
	return false;
}

/* Will print ERR_DATA messages to the DEBUG_SERIAL terminal if PRINT_ERRORS is defined
 */
bool StorageManagerMCB::LogSD(String log_data, Log_Data_Type_t data_type) {
	if (data_type < 0 || data_type >= NUM_LOG_DATA) return false;

	String filename = base_directory + "/";
	filename += log_data_info[data_type].file_prefix;
	filename += log_data_info[data_type].num_files;
	filename += ".txt";

#ifdef PRINT_ERRORS
	if (data_type == ERR_DATA) {
		Serial.println(log_data.c_str());
	}
#endif

	if (!sd_state) {
		return false;
	}

	file = SD.open(filename.c_str(), FILE_WRITE);

	if (!file) {
		Serial.println("Error opening log file");
		return false;
	}

	log_data += "\n";
	String final_log = String((float) millis() / 1000.0f);
	final_log += ',';
	final_log += log_data;

	uint16_t bytes_written = 0;
	uint16_t write_size = final_log.length();
	bytes_written = file.write(final_log.c_str(), write_size);
	file.close();

	// check if the next write should be to a new file
	if (++(log_data_info[data_type].num_writes) == log_data_info[data_type].max_writes) {
		log_data_info[data_type].num_writes = 0;
		log_data_info[data_type].num_files += 1;
	}

	return (bytes_written == write_size);
}

// Basic SD methods -----------------------------------------------------------
bool StorageManagerMCB::StartSD(void) {
	if (!sd_state) {
		if (!SD.begin(BUILTIN_SDCARD)) {
			Serial.println("Unable to start SD card");
			sd_state = false;
		} else {
			Serial.println("Successfully started SD card");
			sd_state = true;
		}
	}

	if (sd_state) {
		if (ConfigureDirectories()) {
			Serial.println("Created base directory for boot");
		} else {
			Serial.println("Error creating base directory");
			sd_state = false;
		}
	}

	return sd_state;
}

bool StorageManagerMCB::ConfigureDirectories(void) {
	if (!SD.exists(LOG_DATA_DIR)) {
		Serial.println("Making log data directory");
		if (!SD.mkdir(LOG_DATA_DIR)) {
			Serial.println("ERR: unable to make log data directory");
			return false;
		}
	}

	if (!SD.exists(FSW_DIR)) {
		Serial.println("Making FSW directory");
		if (!SD.mkdir(FSW_DIR)) {
			Serial.println("ERR: unable to make FSW directory");
			return false;
		}
	}

	base_directory = LOG_DATA_DIR "boot";
	base_directory += String(eeprom_data.boot_count);
	return SD.mkdir(base_directory.c_str());
}

bool StorageManagerMCB::CheckSD(void) {
	return sd_state;
}

bool StorageManagerMCB::RemoveFile(const char * filename) {
	return (sd_state && SD.remove(filename));
}

bool StorageManagerMCB::FileExists(const char * filename) {
	return (sd_state && SD.exists(filename));
}

bool StorageManagerMCB::FileWrite(const char * filename, uint8_t * buffer, uint16_t write_size, bool seek_end, uint16_t position) {
	if (!sd_state) {
		return false;
	}

	file = SD.open(filename, FILE_WRITE);

	if (!file) {
		return false;
	}

	uint16_t bytes_written = 0;
	if (!seek_end) {
		file.seek(position);
	}
	bytes_written = file.write(buffer, write_size);
	file.close();

	return (bytes_written == write_size);
}

uint16_t StorageManagerMCB::FileRead(const char * filename, uint8_t * buffer, uint16_t read_size, uint16_t position) {
	uint16_t bytes_read = 0;

	if (!sd_state) {
		return bytes_read;
	}

	file = SD.open(filename);

	if (file) {
		file.seek(position);
		bytes_read = file.read(buffer, read_size);
		file.close();
	}

	return bytes_read;
}

// Serialization methods ------------------------------------------------------
bool StorageManagerMCB::WriteSD_int32(const char * filename, const int32_t val, bool seek_end, uint16_t position) {
	uint8_t buf[4] = {0, 0, 0, 0};
	buf[0] = (uint8_t) (val >> 24);
	buf[1] = (uint8_t) (val >> 16);
	buf[2] = (uint8_t) (val >> 8);
	buf[3] = (uint8_t) (val);
	return FileWrite(filename, buf, 4, seek_end, position);
}

bool StorageManagerMCB::WriteSD_uint32(const char * filename, const uint32_t val, bool seek_end, uint16_t position) {
	uint8_t buf[4] = {0, 0, 0, 0};
	buf[0] = (uint8_t) (val >> 24);
	buf[1] = (uint8_t) (val >> 16);
	buf[2] = (uint8_t) (val >> 8);
	buf[3] = (uint8_t) (val);
	return FileWrite(filename, buf, 4, seek_end, position);
}

bool StorageManagerMCB::ReadSD_int32(const char * filename, int32_t * result, uint16_t position) {
	uint8_t buf[4] = {0, 0, 0, 0};
	if (FileRead(filename, buf, 4, position) == 4) {
		*result = ((int32_t) buf[0] << 24) | ((int32_t) buf[1] << 16) | ((int32_t) buf[2] << 8) | (int32_t) buf[3];
		return true;
	} else {
		return false;
	}
}

bool StorageManagerMCB::ReadSD_uint32(const char * filename, uint32_t * result, uint16_t position) {
	uint8_t buf[4] = {0, 0, 0, 0};
	if (FileRead(filename, buf, 4, position) == 4) {
		*result = ((uint32_t) buf[0] << 24) | ((uint32_t) buf[1] << 16) | ((uint32_t) buf[2] << 8) | (uint32_t) buf[3];
		return true;
	} else {
		return false;
	}
}

// Basic EEPROM methods -------------------------------------------------------
bool StorageManagerMCB::LoadFromEEPROM(void) {
	// load stored data
	EEPROM.get(EEPROM_BASE_ADDRESS, eeprom_data);

	// increment boot count
	eeprom_data.boot_count += 1;
	EEPROM.put(EEPROM_BASE_ADDRESS + offsetof(EEPROM_Data_t, boot_count), eeprom_data.boot_count);

	// check for a valid eeprom struct version, but don't stop if invalid in case we're in flight
	if (eeprom_data.eeprom_version != EEPROM_VERSION) {
		Serial.println("Invalid EEPROM data version!");
		return false;
	}

	return true;
}

bool StorageManagerMCB::Update_uint8(uint16_t offset, uint8_t data) {
	if (EEPROM_BASE_ADDRESS + offset > EEPROM_MAX_ADDRESS) return false;
	if (offset + sizeof(data) > sizeof(EEPROM_Data_t)) return false;

	// update the software struct
	*(((uint8_t *) &eeprom_data) + offset) = data;

	// update eeprom
	EEPROM.put(EEPROM_BASE_ADDRESS + offset, data);

	return true;
}

bool StorageManagerMCB::Update_uint16(uint16_t offset, uint16_t data) {
	if (EEPROM_BASE_ADDRESS + offset + 1 > EEPROM_MAX_ADDRESS) return false;
	if (offset + sizeof(data) > sizeof(EEPROM_Data_t)) return false;

	// update the software struct
	*((uint16_t *) (((uint8_t *) &eeprom_data) + offset)) = data;

	// update eeprom
	EEPROM.put(EEPROM_BASE_ADDRESS + offset, data);

	return true;
}

bool StorageManagerMCB::Update_uint32(uint16_t offset, uint32_t data) {
	if (EEPROM_BASE_ADDRESS + offset + 3 > EEPROM_MAX_ADDRESS) return false;
	if (offset + sizeof(data) > sizeof(EEPROM_Data_t)) return false;

	// update the software struct
	*((uint32_t *) (((uint8_t *) &eeprom_data) + offset)) = data;

	// update eeprom
	EEPROM.put(EEPROM_BASE_ADDRESS + offset, data);

	return true;
}

bool StorageManagerMCB::Update_float(uint16_t offset, float data) {
	if (EEPROM_BASE_ADDRESS + offset + 3 > EEPROM_MAX_ADDRESS) return false;
	if (offset + sizeof(data) > sizeof(EEPROM_Data_t)) return false;

	// update the software struct
	*((float *) (((uint8_t *) &eeprom_data) + offset)) = data;

	// update eeprom
	EEPROM.put(EEPROM_BASE_ADDRESS + offset, data);

	return true;
}

void StorageManagerMCB::ReconfigureEEPROM() {
	// configuration management
	eeprom_data.eeprom_version = EEPROM_VERSION;

	// for software use
	eeprom_data.boot_count = 0;

	// versioning
	eeprom_data.hardware_version[0] = 'C';
	eeprom_data.hardware_version[1] = '0';
	eeprom_data.software_version    = 0;
	eeprom_data.serial_number       = 2;

	// default motion parameters
	eeprom_data.deploy_velocity      = DEFAULT_FULL_SPEED;
	eeprom_data.deploy_acceleration  = DEFAULT_ACC;
	eeprom_data.retract_velocity     = DEFAULT_FULL_SPEED;
	eeprom_data.retract_acceleration = DEFAULT_ACC;
	eeprom_data.dock_velocity        = DEFAULT_DOCK_SPEED;
	eeprom_data.dock_acceleration    = DEFAULT_ACC;

	// temperature limits (in degrees C)
	eeprom_data.mtr1_temp_hi   = 80.0f;
	eeprom_data.mtr1_temp_lo   = -15.0f;
	eeprom_data.mtr2_temp_hi   = 60.0f;
	eeprom_data.mtr2_temp_lo   = -15.0f;
	eeprom_data.mc1_temp_hi    = 80.0f;
	eeprom_data.mc1_temp_lo    = -40.0f;
	eeprom_data.mc2_temp_hi    = 400.0f;  // not using limits
	eeprom_data.mc2_temp_lo    = -273.0f; // not using limits
	eeprom_data.dcdc_temp_hi   = 400.0f;  // not installed
	eeprom_data.dcdc_temp_lo   = -273.0f; // not installed
	eeprom_data.spare_therm_hi = 400.0f;  // not installed
	eeprom_data.spare_therm_lo = -273.0f; // not installed

	// voltage limits (in Volts)
	eeprom_data.vmon_3v3_hi    = 30.0f; // not using limits
	eeprom_data.vmon_3v3_lo    = -5.0f; // not using limits
	eeprom_data.vmon_15v_hi    = 30.0f; // not using limits
	eeprom_data.vmon_15v_lo    = -5.0f; // not using limits
	eeprom_data.vmon_20v_hi    = 30.0f; // not using limits
	eeprom_data.vmon_20v_lo    = -5.0f; // not using limits
	eeprom_data.vmon_spool_hi  = 30.0f; // not using limits
	eeprom_data.vmon_spool_lo  = -5.0f; // not using limits

	// current limits (in Amps, only very rough estimates, currently inaccurate)
	eeprom_data.imon_brake_hi  = 30.0f;  // not using limits
	eeprom_data.imon_brake_lo  = -30.0f; // not using limits
	eeprom_data.imon_mc_hi     = 30.0f;  // not using limits
	eeprom_data.imon_mc_lo     = -30.0f; // not using limits
	eeprom_data.imon_mtr1_hi   = 13.75f;
	eeprom_data.imon_mtr1_lo   = -10.0f;
	eeprom_data.imon_mtr2_hi   = 30.0f;  // not using limits
	eeprom_data.imon_mtr2_lo   = -30.0f; // not using limits

	// torque limits
	eeprom_data.reel_torque_hi = 500.0f;  // approximately Newtons
	eeprom_data.reel_torque_lo = -500.0f; // approximately Newtons
	eeprom_data.lw_torque_hi   = 2000.0f;  // not using limits
	eeprom_data.lw_torque_lo   = -2000.0f; // not using limits

	// telemetry sample averaging numbers
	eeprom_data.tmslow_num_samples = 60; // should be divisible by the fast number (max value 60)
	eeprom_data.tmfast_num_samples = 10;

	EEPROM.put(EEPROM_BASE_ADDRESS, eeprom_data);
}