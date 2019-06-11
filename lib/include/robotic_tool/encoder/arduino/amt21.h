/*
 * StepperDriver.h
 *
 *  Created on: Jun 18, 2018
 *      Author: biobot
 */

#ifndef AMT21_H_
#define AMT21_H_


#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)

#include <robotic_tool/encoder/amt21_base.h>
#include <robotic_tool/uart/arduino/uart_serial.h>


// When turning the motor clockwise, the encoder gives positive numbers

class AMT21 : public AMT21Base
{
public:

	enum class SercomPort: uint8_t {
		SERCOM_ZERO,
		SERCOM_ONE,
		SERCOM_TWO,
		SERCOM_THREE,
		SERCOM_FOUR,
		SERCOM_FIVE
	};


	AMT21(UartSerial* port, uint32_t baudrate, Rotation rot, AMT21Err* p_err);


private:

	void serial_write(uint8_t _buf[], uint8_t buf_len, AMT21Err* p_err) override;
	void serial_read(uint8_t _buf[], uint8_t buf_len, AMT21Err* p_err) override;
	void delay_ms(uint32_t) override;

	//Sercom* get_sercom(SercomPort sercom_port, AMT21Err* p_err);


	UartSerial* port;
	uint32_t baudrate;
};





#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO









#endif // AMT21_H_
