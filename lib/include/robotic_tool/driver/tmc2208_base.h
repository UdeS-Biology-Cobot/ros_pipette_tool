/*
 * tmc2208_base.h
 *
 *  Created on: Nov 30, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_DRIVER_TMC2208_BASE_H_
#define LIB_BIOBOT_BIOBOT_DRIVER_TMC2208_BASE_H_


#include <stdint.h>

enum class TMC2208Err : uint32_t {
	NONE,
	CRC_MISMATCH,
	MASTER_ADDRESS_MISMATCH,
	REGISTER_UNCHANGED,
	REGISTER_NOT_CLEARED,
	IFCNT_UNCHANGED,
	NOT_IMPLEMENTED,

	READ_TIMEOUT,
	MAX_RETRY_REACHED,

	UART_ERROR,

	INVALID_SERCOMPORT,
	INVALID_DIRECTION,
	INVALID_MRES,
};



// For debugging purposes on SerialUSB
//#define TMC2208_DEBUG


class TMC2208Base
{
public:

	// Read Write field
	enum class RwField : uint8_t {
		READ_ONLY,
		WRITE_ONLY,
		READ_WRITE,
		READ_WRITE_CLEAR
	};

	enum class RotationDir : bool {
		CW = 0,			// Stepper Channel A = Sine, B = Cosine
		CCW = 1,
	};

	enum class LogicLevel : bool {
		LOW_LEVEL,
		HIGH_LEVEL
	};

	enum class MRES : uint8_t {
		TWO_HUNDRED_FIFTY_SIXTH_STEP = 0,
		ONE_HUNDRED_TWENTY_EIGHT_STEP,
		SIXTY_FOURTH_STEP,
		THIRTY_SECOND_STEP,
		SIXTEENTH_STEP,
		EIGHTH_STEP,
		QUARTER_STEP,
		HALF_STEP,
		FULL_STEP,

	};

	enum class INTPOL : bool {
		DISABLED = 0,
		ENABLED = 1,
	};

	enum class MSTEP_REG_SELECT : bool {
		HARDWARE_PINS = 0,
		REGISTER = 1
	};

	enum class PDN_DISABLE : bool {
		ENABLE_STANDSTILL_CURRENT_CONTROL_VIA_PDN_UART_PIN = 0,
		DISABLE_STANDSTILL_CURRENT_CONTROL_VIA_PDN_UART_PIN = 1
	};


	enum class EN_SPREAD_CYCLE : bool {
		STEALTH_CHOP_ENABLED = 0,
		SPREAD_CYCLE_ENABLED = 1
	};


	enum BufferIndex:uint8_t {
		SYNC_IDX,
		ADDRESS_IDX,	// Slave or Master
		REGISTER_IDX,
		DATA0_IDX,		// MSB	**(CRC when a read access is requested)
		DATA1_IDX,		// ...
		DATA2_IDX,		// ...
		DATA3_IDX,		// LSB
		CRC_IDX
	};


	struct Register {
		uint32_t addr;
		RwField rw_field;
		uint32_t data;

	};



	TMC2208Base(uint32_t baudrate);
	virtual ~TMC2208Base() {};
	// GCONF

	void set_direction(RotationDir dir, TMC2208Err* p_err);


	void enable_motor();
	void disable_motor();

	MRES get_max_mres();
	uint32_t get_max_microstep(TMC2208Err* p_err);

	void set_microstep(uint32_t microstep, TMC2208Err* p_err);
	uint32_t get_microstep(TMC2208Err* p_err);



	// IHOLD_IRUN
	void set_ihold(uint8_t ihold, TMC2208Err* p_err);
	uint8_t get_ihold(TMC2208Err* p_err);
	void set_irun(uint8_t ihold, TMC2208Err* p_err);
	uint8_t get_irun(TMC2208Err* p_err);
	void set_iholddelay(uint8_t ihold, TMC2208Err* p_err);
	uint8_t get_iholddelay(TMC2208Err* p_err);

	// CHOPCONF
	void set_mres(MRES mres, TMC2208Err* p_err);
	MRES get_mres(TMC2208Err* p_err);

	void set_intpol(INTPOL intpol, TMC2208Err* p_err);
	INTPOL get_intpol(TMC2208Err* p_err);

	bool err_get(TMC2208Err* p_err);
	void err_clear(TMC2208Err* p_err);



protected:

	virtual void delay_ms(uint32_t) = 0;


	virtual void set_enable_pin(LogicLevel) = 0;
	virtual void set_direction_pin(LogicLevel) = 0;
	virtual void set_halfduplex_pin(LogicLevel) = 0;

	virtual void serial_flush_rx() = 0;
	virtual void serial_flush_tx() = 0;
	virtual void serial_write(const uint8_t *buffer, uint32_t size) = 0;
	virtual void serial_read(uint8_t *buffer, uint32_t size, TMC2208Err* p_err) = 0;

	//virtual uint32_t get_serial_rx_delay() = 0;


	//void pin_init();
	//uint32_t calculate_delay_before_reading(uint32_t baudrate);

protected:

	uint32_t get_serial_rx_delay(uint32_t baudrate);

	// Private because order is important ...
	void set_pdn_disable(PDN_DISABLE pdn_disable, TMC2208Err* p_err);
	PDN_DISABLE get_pdn_disable(TMC2208Err* p_err);
	void set_mstep_reg_select(MSTEP_REG_SELECT mstep_reg_select, TMC2208Err* p_err);
	MSTEP_REG_SELECT get_mstep_reg_select(TMC2208Err* p_err);

	void set_en_spread_cycle(EN_SPREAD_CYCLE en_spread_cycle, TMC2208Err* p_err);
	EN_SPREAD_CYCLE get_en_spread_cycle(TMC2208Err* p_err);


	void set_toff(uint8_t toff, TMC2208Err* p_err);
	uint8_t get_toff(TMC2208Err* p_err);






	// Decoding
	uint32_t decode_dataframe(uint8_t* buf);
	MRES decode_microstep(uint32_t data);
	uint32_t decode_mres(MRES mres, TMC2208Err* p_err);
	MSTEP_REG_SELECT decode_mstep_reg_select(uint32_t data);
	PDN_DISABLE decode_pdn_disable(uint32_t data);
	uint8_t decode_ihold(uint32_t data);
	uint8_t decode_irun(uint32_t data);
	uint8_t decode_iholddelay(uint32_t data);
	INTPOL decode_intpol(uint32_t data);
	TMC2208Base::EN_SPREAD_CYCLE decode_en_spread_cycle(uint32_t data);
	uint8_t decode_toff(uint32_t data);

	// Encoding
	void encode_dataframe(uint8_t* buf, uint32_t data);
	uint32_t encode_mres(MRES mres);
	MRES encode_microstep(uint32_t microstep, TMC2208Err* p_err);
	uint32_t encode_mstep_reg_select(MSTEP_REG_SELECT mstep_reg_select);
	uint32_t encode_pdn_disable(PDN_DISABLE pdn_disable);
	uint32_t encode_ihold(uint8_t ihold);
	uint32_t encode_irun(uint8_t irun);
	uint32_t encode_iholddelay(uint8_t iholddelay);
	uint32_t encode_intpol(INTPOL intpol);
	uint32_t encode_en_spread_cycle(EN_SPREAD_CYCLE en_spread_cycle);
	uint32_t encode_toff(uint8_t toff);

	uint32_t read_register(Register* p_reg, TMC2208Err* p_err);
	//uint32_t read_register(Register* p_reg);
	void write_register(Register* p_reg, uint32_t data, TMC2208Err* p_err);


	void read_all_registers(TMC2208Err* p_err);
	void set_default_config();

	void crc8_atm(uint8_t* datagram, uint8_t len);

	void rw_read_write(Register* p_reg, uint8_t* buf, TMC2208Err* p_err);
	void rw_write_only(Register* p_reg, uint8_t* buf, uint8_t if_counter, TMC2208Err* p_err);
	void rw_read_write_clear(Register* p_reg, uint8_t* buf, TMC2208Err* p_err);



	//Sercom* get_sercom(SercomPort sercom_port, TMC2208Err* p_err);

	//Uart* port;
	//Sercom *sercom;
	//uint32_t baudrate;
	uint32_t SERIAL_BEFORE_READ_DELAY;

	// pin
	/*
	uint32_t pin_half_duplex;
	volatile uint32_t *mode_half_duplex;
	volatile uint32_t *out_half_duplex;

	uint32_t pin_dir;
	volatile uint32_t *mode_dir;
	volatile uint32_t *out_dir;

	uint32_t pin_en;
	volatile uint32_t *mode_en;
	volatile uint32_t *out_en;
*/
	RotationDir direction = RotationDir::CW;


	static const uint8_t READ_ACCESS_REQUEST_BUF_LEN = 4;
	static const uint8_t READ_ACCESS_REPLY_BUF_LEN = 8;
	static const uint8_t WRITE_ACCESS_BUF_LEN = 8;

	const uint8_t MASTER_ADDRESS = 0xFF;

	//uint8_t READ_BM = 0xFE;
	uint8_t RW_WRITE = 0x80;	// Write to register bit



	// Prefill TX buffer with first 2 bytes
	uint8_t tx_buf[WRITE_ACCESS_BUF_LEN] = {
			0x05, 	// Sync
			0x00,	// Slave address
	};

	uint8_t rx_buf[READ_ACCESS_REPLY_BUF_LEN] = {0};

	// GCONF
	const uint8_t EN_SPREAD_CYCLE_BP		=  2;	// n = 1
	const uint8_t PDN_DISABLE_BP			=  6;	// n = 1
	const uint8_t MSTEP_REG_SELECT_BP		=  7;	// n = 1

	const uint32_t EN_SPREAD_CYCLE_BM		=  0x00000002;
	const uint32_t PDN_DISABLE_BM			=  0x00000020;
	const uint32_t MSTEP_REG_SELECT_BM		=  0x00000040;


	// IHOLD_IRUN
	const uint8_t IHOLD_BP					=  0;	// n = 5
	const uint8_t IRUN_BP					=  8;	// n = 5
	const uint8_t IHOLDDELAY_BP				=  16;	// n = 4

	const uint32_t IHOLD_BM					=  0x0000001F;
	const uint32_t IRUN_BM					=  0x00000F80;
	const uint32_t IHOLDDELAY_BM			=  0x00071000;

	// CHOPCONF
	const uint8_t TOFF_BP					=  0;	// n = 4
	const uint8_t MRES_BP					=  24;	// n = 4
	const uint8_t INTPOL_BP					=  28;	// n = 1

	const uint32_t TOFF_BM					=  0x0000000F;
	const uint32_t MRES_BM					=  0x0F000000;
	const uint32_t INTPOL_BM				=  0x10000000;


	// TODO set default driver register values if no opt
	Register GCONF 			= {0x00, RwField::READ_WRITE, 		0};
	Register GSTAT 			= {0x01, RwField::READ_WRITE_CLEAR, 0};
	Register IFCNT 			= {0x02, RwField::READ_ONLY, 		0};
	Register IHOLD_IRUN 	= {0x10, RwField::WRITE_ONLY, 		0x00011F10};
	Register CHOPCONF 		= {0x6C, RwField::READ_WRITE,		0};
	Register DRV_STATUS 	= {0x6F, RwField::READ_WRITE,		0};




	uint8_t SENDDELAY = 8;				// Time that the slave waits to respond (default to 8 bit times)

	uint32_t READ_TIMOUT_MS = 1000;		// Max time for reading a response
	uint32_t MAX_RETRY = 5;				// Number of times to try for a successful response

};


#endif /* LIB_BIOBOT_BIOBOT_DRIVER_TMC2208_BASE_H_ */
