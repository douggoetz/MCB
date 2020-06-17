/*
 *  StorageManagerMCB.h
 *  File defining the class that manages SD storage
 *  Author: Alex St. Clair
 *  March 2018
 *
 *  Note: this class is implemented so that it can be used across multiple
 *        instances. This means that all data members MUST be static.
 */

#ifndef STORAGEMANAGERMCB_H_
#define STORAGEMANAGERMCB_H_

#include "HardwareMCB.h"
#include "SD.h"
#include "WProgram.h"
#include <StdInt.h>
#include <StdLib.h>

// enable/disable printing ERR_DATA messages to DEBUG_SERIAL
#define PRINT_ERRORS	1

// Directories
#define LOG_DATA_DIR	"log_data/"
#define FSW_DIR			"fsw_data/"

// Types of log files
enum Log_Data_Type_t {
	VMON_DATA = 0,
	IMON_DATA,
	TEMP_DATA,
	ERR_DATA,
	MOTION_DATA,
	// add new data types here
	NUM_LOG_DATA
};

struct Log_Data_Info_t {
	uint8_t num_writes; // tracks number of writes to current file
	uint8_t max_writes; // maximum number of writes per file for this type of file
	uint16_t num_files; // tracks number of log files since boot
	String file_prefix; // filename prefix for this type of file
};

class StorageManagerMCB {
public:
	// Empty constructor, will be called multiple times! Once for each instance
	StorageManagerMCB() { };
	~StorageManagerMCB() { };

	// Methods for logging data to SD (StorageManagerMCB handles file naming)
	bool LogSD(uint8_t * log_data, int data_size, Log_Data_Type_t data_type);
	bool LogSD(String log_data, Log_Data_Type_t data_type); // string should not end with newline char

	// Basic SD methods
	bool StartSD(uint32_t boot_num); // must be called before use of file I/O (but after configManager.Initialize())
	bool CheckSD(void);
	bool ConfigureDirectories(void);
	bool RemoveFile(const char * filename);
	bool FileExists(const char * filename);
	bool FileWrite(const char * filename, uint8_t * buffer, uint16_t write_size, bool seek_end=true, uint16_t position=0);
	uint16_t FileRead(const char * filename, uint8_t * buffer, uint16_t read_size, uint16_t position=0);

	// Serialized SD writes/reads
	bool WriteSD_int32(const char * filename, const int32_t val, bool seek_end=true, uint16_t position=0);
	bool WriteSD_uint32(const char * filename, const uint32_t val, bool seek_end=true, uint16_t position=0);
	bool ReadSD_int32(const char * filename, int32_t * result, uint16_t position=0);
	bool ReadSD_uint32(const char * filename, uint32_t * result, uint16_t position=0);

	// base directory for data from this boot
	static String base_directory;

	// table of data for each log data type
	static Log_Data_Info_t log_data_info[NUM_LOG_DATA];

private:
	static bool sd_state;
	static uint32_t boot_number;
	File file; // static exception: each instance has a file so it isn't constantly declaring one
};

#endif