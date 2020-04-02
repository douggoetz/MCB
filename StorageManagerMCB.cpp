/*
 *  StorageManagerMCB.cpp
 *  File implementing the class that manages SD storage
 *  Author: Alex St. Clair
 *  March 2018
 *
 *  Note: this class is implemented so that it can be used across multiple
 *        instances. This means that all data members MUST be static.
 */

#include "StorageManagerMCB.h"

bool StorageManagerMCB::sd_state = false;
uint32_t StorageManagerMCB::boot_number = 0;
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
bool StorageManagerMCB::StartSD(uint32_t boot_num) {
	boot_number = boot_num;

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
	base_directory += String(boot_number);
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