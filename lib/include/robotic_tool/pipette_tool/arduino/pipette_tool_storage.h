/*
 * storage.h
 *
 *  Created on: Nov 14, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_STORAGE_ARDUINO_STORAGE_H_
#define LIB_BIOBOT_BIOBOT_STORAGE_ARDUINO_STORAGE_H_

#include <stdint.h>

enum class PipetteToolStorageErr : uint8_t {
	NONE,
	INIT_FAILED,
	OPEN_FILE_FAILED,
	FILE_NOT_FOUND,
	DELETE_FILE_FAILED,
	WRITE_ERROR,
	READ_ERROR,
	RENAME_FILE_FAILED,
	INVALID_DATASTORAGE,
	INVALID_DATASTORAGE_TYPE,
};



#if defined(ARDUINO)


#include <SdFat.h>


class PipetteToolStorage  {
public:



	enum class DataStorageType : uint8_t {
		BOOL,
		FLOAT,
		DOUBLE,
		UINT8_T,
		UINT16_T,
		UINT32_T,
		UINT64_T,
		INT8_T,
		INT16_T,
		INT32_T,
		INT64_T
	};


	// To store a new data
	// Add to DataStorage and DataStorageStruct
	enum DataStorage : uint8_t {
		PT_TYPE = 0,
		PT_SN,
		PT_LEAD,
		EP_TOP,
		EP_INIT,
		EP_TIP,
		EP_BOT,
		EP_MIN,
		EP_MAX,



		SIZE	// MUST be last
	};

	struct DataStorageStruct {
		const char* name;
		const char* filename;
		DataStorageType storage_type;
		bool action_req;	//
	};

	union storage_type {
    	bool b;
		float f;
		double d;
		uint8_t u8;
		uint16_t u16;
		uint32_t u32;
		uint64_t u64;
		int8_t i8;
		int16_t i16;
		int32_t i32;
		int64_t i64;

    	uint8_t bytes[sizeof(u64)];
	};

	// FILENAME MUST be lower then 10 caracters and MUST be unique
	const DataStorageStruct data_storage[DataStorage::SIZE] = {
	//	 NAME							FILENAME		DATA_TYPE
		{"Type", 					"PT_TYPE",		DataStorageType::UINT8_T,	false},
		{"Serial number", 			"PT_SN", 		DataStorageType::UINT32_T,	false},
		{"Lead", 					"PT_LEAD", 		DataStorageType::DOUBLE,	false},
		{"Top encoder pos", 		"EP_TOP", 		DataStorageType::INT32_T,	true},
		{"Init encoder pos", 		"EP_INIT", 		DataStorageType::INT32_T,	true},
		{"Tip encoder pos", 		"EP_TIP", 		DataStorageType::INT32_T,	true},
		{"Bottom encoder pos", 		"EP_BOT", 		DataStorageType::INT32_T,	true},
		{"Min init encoder pos", 	"EP_MIN", 		DataStorageType::INT32_T,	false},
		{"Max init encoder pos", 	"EP_MAX", 		DataStorageType::INT32_T,	false},
	};


	//static_assert(std::is_same<typename std::underlying_type<PipetteTool::Type>::type, storage_type.b >::value, "Wrong type for : PipetteTool::Type enum");



	PipetteToolStorage();

	// All available data
	void store_data(DataStorage ds, bool data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, float data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, double data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, uint8_t data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, uint16_t data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, uint32_t data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, uint64_t data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, int8_t data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, int16_t data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, int32_t data, PipetteToolStorageErr* p_err);
	void store_data(DataStorage ds, int64_t data, PipetteToolStorageErr* p_err);



	void retrieve_data(DataStorage ds, bool* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, float* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, double* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, uint8_t* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, uint16_t* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, uint32_t* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, uint64_t* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, int8_t* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, int16_t* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, int32_t* data, PipetteToolStorageErr* p_err);
	void retrieve_data(DataStorage ds, int64_t* data, PipetteToolStorageErr* p_err);


	bool err_get(PipetteToolStorageErr* p_err);
	void err_clear(PipetteToolStorageErr* p_err);

private:

	// https://stackoverflow.com/a/25668287
	// (4) every character type occupies exactly 1 byte. So, arrays of a character
	// type can be used to "read" the bytes of any other object in a union member.
	// However, there is undefined behaviour when accessing members of atomic unions (or structs, also).


	void sd_store(DataStorage ds, uint8_t buf[], size_t buf_len, PipetteToolStorageErr* p_err);
	void sd_retrieve(DataStorage ds, uint8_t buf[], size_t buf_len, PipetteToolStorageErr* p_err);

	uint32_t get_data_storage_idx(DataStorage ds, PipetteToolStorageErr* p_err);
	uint32_t get_buf_len(DataStorageType dst);
	void validate_storage_type(DataStorage ds, DataStorageType dst, PipetteToolStorageErr* p_err);
	File src_file;
	File tmp_file;


	// Reserved
	const char* tmp_filename = "tmp";		// PipetteTool::Type


	SdFat sd;
};









#endif	// ARDUINO




#endif /* LIB_BIOBOT_BIOBOT_STORAGE_ARDUINO_STORAGE_H_ */
