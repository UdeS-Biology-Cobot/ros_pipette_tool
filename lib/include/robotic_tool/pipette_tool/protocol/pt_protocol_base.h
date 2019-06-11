/*
 * pt_protocol_base.h
 *
 *  Created on: Dec 5, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_PROTOCOL_PT_PROTOCOL_BASE_H_
#define LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_PROTOCOL_PT_PROTOCOL_BASE_H_

#include <stdint.h>




enum class PtProtocolErr : uint32_t {
	NONE,

	PIPETTE_TOOL_ERROR,
	PROTOCOL_ERROR,

	TRANSMIT_ERROR,
	RECEIVE_ERROR,
	BAD_CRC,
};







class PtProtocolBase
{
public:



	enum class Direction : bool {
		ASPIRATE,
		DISPENSE
	};


	enum State : uint8_t {
		SYNC,
		SLAVE,
		MSG_LENGTH,
		REGISTER,
		DATA_REGISTER,
		DATA_REGISTER_STORE,
		CRC
	};

	// Read Write field
	enum class RwField : uint8_t {
		READ_ONLY,
		WRITE_ONLY,
		READ_WRITE,
		CLEAR_UPON_READ
	};



	enum class DataType : uint8_t {
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


	enum class CmdIdx : uint8_t {
		CMD_SUCCESSFUL,
		ERROR_PT,
		ERROR_PT_PROTOCOL,
		ASPIRATE,
		DISPENSE,
		MULTI_SEQUENCE,
		HOMING,
		EJECT_TIP,
		MAX_NL,
		MAX_SPEED,
		OFFSET_NL,
		REM_ASPIRATE_NL,
		REM_DISPENSE_NL,
		SERIAL_NUMBER,
		MOVE_TO_TOP,
		MOVE_TO_TIP,

		SIZE
	};

	enum class DataRegIdx : uint8_t {
		PT_ERROR,
		PT_PROTOCOL_ERROR,
		SPEED,
		NANO_LITERS,
		SIGNED_NANO_LITERS,
		DIRECTION,
		SEQUENCE_NUMBER,
		SERIAL_NUMBER,

		SIZE
	};

	struct Command {
		CmdIdx idx;
		RwField rw_field;
		uint32_t data;
	};

	struct DataReg {
		DataRegIdx addr;
		DataType data_type;
		uint8_t size;
	};



	void crc8_atm(uint8_t* datagram, uint8_t datagramLength);

	//void store_data_to_buf(uint8_t _tx_buf[], uint8_t data_buf[], uint8_t buf_len);
	bool err_get(PtProtocolErr* p_err);
	void err_clear(PtProtocolErr* p_err);

	// TODO set default driver register values if no opt
	/*
	Command ERROR 				= {CmdIdx::ERROR, 			RwField::READ_WRITE, 	0};
	Command ASPIRATE 			= {CmdIdx::ASPIRATE, 		RwField::WRITE_ONLY, 	0};
	Command DISPENSE 			= {CmdIdx::DISPENSE, 		RwField::WRITE_ONLY, 	0};
	Command MULTI_SEQUENCE 		= {CmdIdx::MULTI_SEQUENCE, 	RwField::WRITE_ONLY,	0};
	Command HOMING 				= {CmdIdx::HOMING, 			RwField::WRITE_ONLY,	0};
	Command EJECT_TIP 			= {CmdIdx::EJECT_TIP, 		RwField::WRITE_ONLY,	0};
	Command MAX_NL 				= {CmdIdx::MAX_NL, 			RwField::READ_ONLY,		0};
	Command SERIAL_NUMBER 		= {CmdIdx::SERIAL_NUMBER, 	RwField::READ_ONLY,		0};
*/

	DataReg data_reg[(uint32_t)DataRegIdx::SIZE] = {
			{DataRegIdx::PT_ERROR,				DataType::UINT32_T, sizeof(uint32_t)},
			{DataRegIdx::PT_PROTOCOL_ERROR,		DataType::UINT32_T, sizeof(uint32_t)},
			{DataRegIdx::SPEED, 				DataType::DOUBLE, 	sizeof(double)},
			{DataRegIdx::NANO_LITERS, 			DataType::UINT32_T, sizeof(uint32_t)},
			{DataRegIdx::SIGNED_NANO_LITERS, 	DataType::INT32_T, sizeof(int32_t)},
			{DataRegIdx::DIRECTION,				DataType::BOOL, 	sizeof(bool)},
			{DataRegIdx::SEQUENCE_NUMBER,		DataType::UINT32_T, sizeof(uint32_t)},
			{DataRegIdx::SERIAL_NUMBER,			DataType::UINT32_T, sizeof(uint32_t)}
	};


	const uint8_t SYNC_FRAME	= 0xAA;
	const uint8_t MASTER_FRAME = 0xFF;


};








#endif /* LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_PROTOCOL_PT_PROTOCOL_BASE_H_ */
