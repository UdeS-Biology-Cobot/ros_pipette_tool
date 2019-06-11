/*
 * storage.cpp
 *
 *  Created on: Nov 14, 2018
 *      Author: biobot
 */




#if defined(ARDUINO)


#include <robotic_tool/pipette_tool/arduino/pipette_tool_storage.h>
#include <type_traits>

PipetteToolStorage::PipetteToolStorage()
{

}






/*
void PipetteToolStorage::store_pt_type(PipetteTool::Type type)
{
	storage_type store_val;






	// Match type of
	store_val.u8 = type;

	store_data(store_val.bytes, sizeof(store_val.u8), pt_type_filename);

	// https://en.cppreference.com/w/cpp/types/underlying_type
	static_assert(std::is_same<typename std::underlying_type<decltype(type)>::type, decltype(store_val.u8)>::value, "Wrong type for : PipetteTool::Type enum");
}


PipetteTool::Type PipetteToolStorage::retrieve_pt_type()
{
	storage_type store_val;

	retrieve_data(store_val.bytes, sizeof(store_val.u8), pt_type_filename);

	return (PipetteTool::Type)store_val.u8;
}


void PipetteToolStorage::store_pt_serialnumber(uint32_t sn)
{
	storage_type store_val;

	// Match type of
	store_val.u32 = sn;
	store_data(store_val.bytes, sizeof(store_val.u32), pt_serialnumber_filename);

	static_assert(std::is_same<decltype(sn), decltype(this->serial_number)>::value, "Wrong type for : Serial number");
	static_assert(std::is_same<decltype(sn), decltype(store_val.u32)>::value, "Wrong type for : Serial number");
}

uint32_t PipetteToolStorage::retrieve_pt_serialnumber()
{
	storage_type store_val;

	retrieve_data(store_val.bytes, sizeof(store_val.u32), pt_serialnumber_filename);

	return store_val.u32;
}
*/



void PipetteToolStorage::sd_store(DataStorage ds, uint8_t buf[], size_t buf_len, PipetteToolStorageErr* p_err)
{
	uint32_t idx = get_data_storage_idx(ds, p_err);
	if(err_get(p_err)) {return;}

	const char* src_filename = data_storage[idx].filename;


	//bool is_open_srcfile = false;
	bool is_open_tmpfile = false;

	// Initialise sd card
	if (!sd.begin(SDCARD_SS_PIN)) {
		*p_err = PipetteToolStorageErr::INIT_FAILED;
		goto error;
	}

	// Verify if the tmp file exists, if so delete it
	if (sd.exists(tmp_filename)) {
		if (!sd.remove(tmp_filename)) {
			*p_err = PipetteToolStorageErr::DELETE_FILE_FAILED;
			goto error;
		}
	}

	// Open tmp file
	tmp_file = sd.open(tmp_filename, FILE_WRITE);
	if(!tmp_file) {
		*p_err = PipetteToolStorageErr::OPEN_FILE_FAILED;
		goto error;
	}
	is_open_tmpfile = true;


	// Write buffer to tmp file
	tmp_file.write(buf, buf_len);

	// Check for errors
	if (tmp_file.getWriteError()) {
		*p_err = PipetteToolStorageErr::WRITE_ERROR;
		goto error;
	}

	// Close tmp file
	tmp_file.close();
	is_open_tmpfile = false;



	// Check if src file exists
	if (sd.exists(src_filename)) {

		if (!sd.remove(src_filename)) {
			*p_err = PipetteToolStorageErr::DELETE_FILE_FAILED;
			goto error;
		}
	}

	// Rename
	if (!sd.rename(tmp_filename, src_filename)) {
		*p_err = PipetteToolStorageErr::RENAME_FILE_FAILED;
		goto error;
	}


	return;

// If there is an error close communication
error:

	if (is_open_tmpfile) {
		tmp_file.close();
	}

	return;
}


void PipetteToolStorage::sd_retrieve(DataStorage ds, uint8_t buf[], size_t buf_len, PipetteToolStorageErr* p_err)
{

	uint32_t idx = get_data_storage_idx(ds, p_err);
	if(err_get(p_err)) {return;}

	const char* src_filename = data_storage[idx].filename;



	//bool is_open_srcfile = false;
	bool is_open_srcfile = false;
	size_t rx_bytes;

	// Initialise sd card
	if (!sd.begin(SDCARD_SS_PIN)) {
		*p_err = PipetteToolStorageErr::INIT_FAILED;
		goto error;
	}

	// Open file
	src_file = sd.open(src_filename, FILE_READ);
	if(!src_file) {
		*p_err = PipetteToolStorageErr::OPEN_FILE_FAILED;
		goto error;
	}
	is_open_srcfile = true;

	// Read data
	rx_bytes = src_file.readBytes(buf, buf_len);

	if (rx_bytes != buf_len) {
		*p_err = PipetteToolStorageErr::READ_ERROR;
		goto error;
	}

	src_file.close();

	return;

// If there is an error close communication
error:

	if (is_open_srcfile) {
		src_file.close();
	}

	return;
}






uint32_t PipetteToolStorage::get_data_storage_idx(DataStorage ds, PipetteToolStorageErr* p_err)
{
	if (ds >= DataStorage::SIZE) {
		*p_err = PipetteToolStorageErr::INVALID_DATASTORAGE;
	}

	return ds;
}

/*
uint32_t PipetteToolStorage::get_buf_len(DataStorageType dst)
{
	switch (dst) {
		case DataStorageType::BOOL:
			return (uint32_t)PT_TYPE;

	}
}
*/


void PipetteToolStorage::validate_storage_type(DataStorage ds, DataStorageType dst, PipetteToolStorageErr* p_err)
{
	uint32_t idx = get_data_storage_idx(ds, p_err);
	if(err_get(p_err)) {return;}

	if (data_storage[idx].storage_type != dst) {
		*p_err = PipetteToolStorageErr::INVALID_DATASTORAGE_TYPE;
	}
}



void PipetteToolStorage::store_data(DataStorage ds, bool data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.b = data};
	validate_storage_type(ds, DataStorageType::BOOL, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.b), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, float data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.f = data};
	validate_storage_type(ds, DataStorageType::FLOAT, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.f), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, double data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.d = data};
	validate_storage_type(ds, DataStorageType::DOUBLE, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.d), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, uint8_t data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.u8 = data};
	validate_storage_type(ds, DataStorageType::UINT8_T, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.u8), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, uint16_t data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.u16 = data};
	validate_storage_type(ds, DataStorageType::UINT16_T, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.u16), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, uint32_t data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.u32 = data};
	validate_storage_type(ds, DataStorageType::UINT32_T, p_err);
	sd_store(ds, st.bytes, sizeof(st.u32), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, uint64_t data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.u64 = data};
	validate_storage_type(ds, DataStorageType::UINT64_T, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.u64), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, int8_t data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.i8 = data};
	validate_storage_type(ds, DataStorageType::INT8_T, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.i8), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, int16_t data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.i16 = data};
	validate_storage_type(ds, DataStorageType::INT16_T, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.i16), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, int32_t data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.i32 = data};
	validate_storage_type(ds, DataStorageType::INT32_T, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.i32), p_err);
}

void PipetteToolStorage::store_data(DataStorage ds, int64_t data, PipetteToolStorageErr* p_err)
{
	storage_type st = {.i64 = data};
	validate_storage_type(ds, DataStorageType::INT64_T, p_err);
	if(err_get(p_err)) {return;}
	sd_store(ds, st.bytes, sizeof(st.i64), p_err);
}






void PipetteToolStorage::retrieve_data(DataStorage ds, bool* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::BOOL, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.b), p_err);
	*data = st.b;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, float* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::FLOAT, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.f), p_err);
	*data = st.f;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, double* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::DOUBLE, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.d), p_err);
	*data = st.d;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, uint8_t* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::UINT8_T, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.u8), p_err);
	*data = st.u8;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, uint16_t* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::UINT16_T, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.u16), p_err);
	*data = st.u16;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, uint32_t* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::UINT32_T, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.u32), p_err);
	*data = st.u32;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, uint64_t* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::UINT64_T, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.u64), p_err);
	*data = st.u64;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, int8_t* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::INT8_T, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.i8), p_err);
	*data = st.i8;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, int16_t* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::INT16_T, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.i16), p_err);
	*data = st.i16;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, int32_t* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::INT32_T, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.i32), p_err);
	*data = st.i32;
}

void PipetteToolStorage::retrieve_data(DataStorage ds, int64_t* data, PipetteToolStorageErr* p_err)
{
	storage_type st;
	validate_storage_type(ds, DataStorageType::INT64_T, p_err);
	if(err_get(p_err)) {return;}
	sd_retrieve(ds, st.bytes, sizeof(st.i64), p_err);
	*data = st.i64;
}






bool PipetteToolStorage::err_get(PipetteToolStorageErr* p_err)
{
	if (*p_err == PipetteToolStorageErr::NONE) {
		return false;
	}
	return true;
}

void PipetteToolStorage::err_clear(PipetteToolStorageErr* p_err)
{
	*p_err = PipetteToolStorageErr::NONE;
}












#endif	// ARDUINO
