/*
 * amt21_base.h
 *
 *  Created on: Nov 30, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_ENCODER_AMT21_BASE_H_
#define LIB_BIOBOT_BIOBOT_ENCODER_AMT21_BASE_H_

#include <stdint.h>


enum class AMT21Err : uint32_t {
	NONE,
	CRC_MISMATCH,
	READ_TIMEOUT,
	MAX_RETRY_REACHED,
	UART_ERROR,
	RESET_ERROR,

	INVALID_SERCOMPORT
};



// When turning the motor clockwise, the encoder gives positive numbers

class AMT21Base
{
public:
	// Set Encoder direction for positive position
	enum class Rotation : bool {
		CW,
		CCW
	};

	AMT21Base(Rotation rot);
	virtual ~AMT21Base() {};

	uint32_t get_resolution();
	int32_t read(AMT21Err* p_err);						// Read encoder
	int32_t read(int32_t offset, AMT21Err* p_err);		// Read encoder and apply offset to it
	void reset(AMT21Err* p_err);						// Turn counter resets to 0

	bool err_get(AMT21Err* p_err);
	void err_clear(AMT21Err* p_err);

protected:
	// OS dependant functions
	virtual void delay_ms(uint32_t) = 0;
	virtual void serial_write(uint8_t buf[], uint8_t buf_len, AMT21Err* p_err) = 0;
	virtual void serial_read(uint8_t buf[], uint8_t buf_len, AMT21Err* p_err) = 0;

	const uint32_t RESOLUTION_14_BIT = 16384;

	uint16_t read_single_turn_position(AMT21Err* p_err);
	int16_t read_turns_counter(AMT21Err* p_err);


	uint16_t decode_single_turn_position(uint8_t _buf[]);
	int16_t decode_turns_counter(uint8_t _buf[]);

	bool crc(uint8_t _buf[], AMT21Err* p_err);

	void sr(uint8_t _tx_buf[], uint32_t _tx_buf_len, uint8_t _rx_buf[], uint32_t _rx_buf_len, AMT21Err* p_err);
	const uint8_t CMD_READ_SINGLE_TURN_POSITION = 0x54;
	const uint8_t CMD_READ_TURNS_COUNTER = 0x55;
	const uint8_t CMD_SEQ_RESET_ENCODER[3] = {0x34, 0xA5, 0x7E};


	uint8_t tx_buf[4];
	uint8_t rx_buf[4];

	uint32_t RESET_TIMEOUT_MS = 500;	// Typical start-up time is 200 ms
	uint32_t READ_TIMOUT_MS = 1000;
	uint32_t MAX_RETRY = 5;


	const int32_t MAX_PULSE_INIT_ERROR = 20;

	Rotation rot;
};



#endif /* LIB_BIOBOT_BIOBOT_ENCODER_AMT21_BASE_H_ */
