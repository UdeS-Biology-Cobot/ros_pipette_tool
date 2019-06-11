/*
 * amt21_interface.cpp
 *
 *  Created on: Nov 30, 2018
 *      Author: biobot
 */


#if defined(ARDUINO)

#include <robotic_tool/pipette_tool/interface/encoder/amt21_interface.h>

AMT21Interface::AMT21Interface(UartSerial* port, uint32_t baudrate, Rotation rot, EncoderInterfaceErr* p_err)
: AMT21(port, baudrate, rot, (AMT21Err *)p_err)
{
	if(AMT21::err_get((AMT21Err *)p_err)) {
		*p_err = modify_err_code((AMT21Err *)p_err);
	}
}

uint32_t AMT21Interface::get_resolution()
{
	return AMT21::get_resolution();
}

int32_t AMT21Interface::read(EncoderInterfaceErr* p_err)
{
	AMT21Err err_amt21 = AMT21Err::NONE;

	int32_t pos = AMT21::read(&err_amt21);
	if(AMT21::err_get(&err_amt21)) {
		*p_err = modify_err_code(&err_amt21);
	}

	return pos;
}

int32_t AMT21Interface::read(int32_t offset, EncoderInterfaceErr* p_err)
{
	AMT21Err err_amt21 = AMT21Err::NONE;

	int32_t pos = AMT21::read(offset, &err_amt21);
	if(AMT21::err_get(&err_amt21)) {
		*p_err = modify_err_code(&err_amt21);
	}

	return pos;
}

void AMT21Interface::reset(EncoderInterfaceErr* p_err)
{
	AMT21Err err_amt21 = AMT21Err::NONE;

	AMT21::reset(&err_amt21);
	if(AMT21::err_get(&err_amt21)) {
		*p_err = modify_err_code(&err_amt21);
	}
}



EncoderInterfaceErr AMT21Interface::modify_err_code(AMT21Err* p_err)
{
	switch (*p_err) {
		case AMT21Err::NONE:
			return EncoderInterfaceErr::NONE;

		case AMT21Err::READ_TIMEOUT:
		case AMT21Err::MAX_RETRY_REACHED:
			return EncoderInterfaceErr::TIMEOUT;
		case AMT21Err::UART_ERROR:
		case AMT21Err::RESET_ERROR:
		case AMT21Err::CRC_MISMATCH:
		case AMT21Err::INVALID_SERCOMPORT:
			return EncoderInterfaceErr::INTERNAL_ERROR;

		default:
			return EncoderInterfaceErr::INTERNAL_ERROR;
	}
}


bool AMT21Interface::err_get(EncoderInterfaceErr* p_err)
{
	if (*p_err == EncoderInterfaceErr::NONE) {
		return false;
	}
	return true;
}

void AMT21Interface::err_clear(EncoderInterfaceErr* p_err)
{
	*p_err = EncoderInterfaceErr::NONE;
}

#endif	// ARDUINO


