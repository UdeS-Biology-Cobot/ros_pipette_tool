/*
 * StepperDriver.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: biobot
 */




#include <robotic_tool/encoder/amt21_base.h>




AMT21Base::AMT21Base(Rotation rot)
: rot(rot)
{
}




// Get position with offset
int32_t AMT21Base::read(int32_t offset, AMT21Err* p_err)
{
	int32_t pos = read(p_err);
	pos += offset;
	return pos;
}


int32_t AMT21Base::read(AMT21Err* p_err)
{
	int16_t turn_number;
	uint16_t single_turn_position;
	int32_t position;

	single_turn_position = read_single_turn_position(p_err);
	if (err_get(p_err)) {return 0;}

	turn_number = read_turns_counter(p_err);
	if (err_get(p_err)) {return 0;}

	position = turn_number * RESOLUTION_14_BIT;

	if (position < 0) {
		position -= ((RESOLUTION_14_BIT - single_turn_position)-RESOLUTION_14_BIT);
	} else {
		position += single_turn_position;
	}

	if (rot == Rotation::CCW) {
		position *= -1;
	}

	return position;
}


int16_t AMT21Base::read_turns_counter(AMT21Err* p_err)
{
	tx_buf[0] = CMD_READ_TURNS_COUNTER;

	sr(tx_buf, 1, rx_buf, 2, p_err);
	if (err_get(p_err)) {return 0;}

	int16_t turn_number = decode_turns_counter(rx_buf);
	return turn_number;
}

uint16_t AMT21Base::read_single_turn_position(AMT21Err* p_err)
{
	tx_buf[0] = CMD_READ_SINGLE_TURN_POSITION;

	sr(tx_buf, 1, rx_buf, 2, p_err);
	if (err_get(p_err)) {return 0;}

	uint16_t single_turn_position = decode_single_turn_position(rx_buf);
	return single_turn_position;
}



uint16_t AMT21Base::decode_single_turn_position(uint8_t _buf[])
{
	uint16_t single_turn_position = _buf[0] + ((_buf[1] & 0x3F) << 8);
	return single_turn_position;
}

int16_t AMT21Base::decode_turns_counter(uint8_t _buf[])
{
	int16_t turn_counter = static_cast<int16_t>(_buf[0] + ((_buf[1] & 0x3F) << 8));

	// Convert to 14 bit signed
	if (turn_counter >= 8192) {
		turn_counter -= 16384;
	}

	return turn_counter;
}

void AMT21Base::sr(uint8_t _tx_buf[], uint32_t _tx_buf_len, uint8_t _rx_buf[], uint32_t _rx_buf_len, AMT21Err* p_err)
{
	AMT21Err local_err = AMT21Err::NONE;
	uint32_t retry_ctr;

	for (retry_ctr = 0; retry_ctr < MAX_RETRY; retry_ctr++) {
		local_err = AMT21Err::NONE;

		serial_write(_tx_buf, _tx_buf_len, &local_err);

		if (!err_get(&local_err)) {
			serial_read(_rx_buf, _rx_buf_len, &local_err);
		}

		if (!err_get(&local_err)) {
			break;
		}
	}

	if (retry_ctr == MAX_RETRY) {
		*p_err = AMT21Err::MAX_RETRY_REACHED;
#if defined(AMT21_DEBUG)
		Serial.println("--- ERROR : MAX_RETRY_REACHED for read");
#endif
	}

}


bool AMT21Base::err_get(AMT21Err* p_err)
{
	if (*p_err == AMT21Err::NONE) {
		return false;
	}
	return true;
}

void AMT21Base::err_clear(AMT21Err* p_err)
{
	*p_err = AMT21Err::NONE;
}



uint32_t AMT21Base::get_resolution()
{
	return RESOLUTION_14_BIT;
}

void AMT21Base::reset(AMT21Err* p_err)
{
	for (int i = 0; i < 3; i++) {
		tx_buf[i] = CMD_SEQ_RESET_ENCODER[i];
	}
	serial_write(tx_buf, 3, p_err);
	if (err_get(p_err)) {return;}

	delay_ms(RESET_TIMEOUT_MS);

	int32_t pos = read(p_err);
	if (err_get(p_err)) {return;}

	if (pos > (int32_t)RESOLUTION_14_BIT + MAX_PULSE_INIT_ERROR || pos < -MAX_PULSE_INIT_ERROR) {
		*p_err = AMT21Err::RESET_ERROR;
		return;
	}
}



bool AMT21Base::crc(uint8_t _buf[], AMT21Err* p_err){

	bool crc_valid = false;

	bool L0 = _buf[0] & 0x01;
	bool L1 = (_buf[0] & 0x02) >> 1;
	bool L2 = (_buf[0] & 0x04) >> 2;
	bool L3 = (_buf[0] & 0x08) >> 3;
	bool L4 = (_buf[0] & 0x10) >> 4;
	bool L5 = (_buf[0] & 0x20) >> 5;
	bool L6 = (_buf[0] & 0x40) >> 6;
	bool L7 = (_buf[0] & 0x80) >> 7;

	bool H0 = _buf[1] & 0x01;
	bool H1 = (_buf[1] & 0x02) >> 1;
	bool H2 = (_buf[1] & 0x04) >> 2;
	bool H3 = (_buf[1] & 0x08) >> 3;
	bool H4 = (_buf[1] & 0x10) >> 4;
	bool H5 = (_buf[1] & 0x20) >> 5;

	bool K0 = (_buf[1] & 0x40) >> 6;
	bool K1 = (_buf[1] & 0x80) >> 7;


	if (K1 == !(H5^H3^H1^L7^L5^L3^L1) &&
	    K0 == !(H4^H2^H0^L6^L4^L2^L0))
	{
		crc_valid = true;
	} else {
		*p_err = AMT21Err::CRC_MISMATCH;
#if defined(AMT21_DEBUG)
		Serial.print("CRC_MISMATCH ");
#endif
	}

	return crc_valid;
}


