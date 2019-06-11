/*
 * StepperDriver.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: biobot
 */



#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)

#include <robotic_tool/encoder/arduino/amt21.h>
#include <Arduino.h>




AMT21::AMT21(UartSerial* port, uint32_t baudrate, Rotation rot, AMT21Err* p_err)
: AMT21Base(rot),
  port(port),
  baudrate(baudrate)
{
	port->begin(baudrate);
	//while(!port);

	port->set_halfduplex_pin(1);

	delayMicroseconds(500);	// Wait for TX pin to go high (Calculated for low baudrate and RC filter)
}

/*
Sercom* AMT21::get_sercom(SercomPort sercom_port, AMT21Err* p_err) {

	switch (sercom_port) {
		case SercomPort::SERCOM_ZERO:
			return SERCOM0;
			break;
		case SercomPort::SERCOM_ONE:
			return SERCOM1;
			break;
		case SercomPort::SERCOM_TWO:
			return SERCOM2;
			break;
		case SercomPort::SERCOM_THREE:
			return SERCOM3;
			break;
		case SercomPort::SERCOM_FOUR:
			return SERCOM4;
			break;
		case SercomPort::SERCOM_FIVE:
			return SERCOM5;
			break;
		default:
			*p_err = AMT21Err::INVALID_SERCOMPORT;
#if defined(AMT21_DEBUG)
			SerialUSB.print("INVALID_SERCOMPORT ");
			SerialUSB.println((uint8_t)sercom_port);
#endif
			return nullptr;
			break;
	}
}
*/



void AMT21::serial_write(uint8_t _buf[], uint8_t buf_len, AMT21Err* p_err)
{
	uint8_t byte_ctr = 0;

	// Flush RX buffer
	port->flush_rx();


	// Write byte to buffer
	while (byte_ctr != buf_len) {
		byte_ctr += port->write(_buf, buf_len);
	}

	// Wait till last stop bit of buffer have been transmitted
	// There is a bug in method flush();
	port->flush_tx();

	// Enable Receiver-enable input
	port->set_halfduplex_pin(0);
}



void AMT21::delay_ms(uint32_t _delay)
{
	delay(_delay);
}






void AMT21::serial_read(uint8_t _buf[], uint8_t buf_len, AMT21Err* p_err)
{
	uint8_t byte_ctr = 0;


	// Let the rx buffer fill before checking
	// 10 bit @ 2 Mbps = 5 microseconds
	// latency max = 29 usec
	delayMicroseconds(5*buf_len + 29);


	uint32_t prevMillis = millis();
	uint32_t currMillis;
	uint32_t diffMillis;

	UartSerialErr err_uart = UartSerialErr::NONE;

	while (byte_ctr != buf_len) {



		if (port->available() > 0) {

			_buf[byte_ctr] = port->read(&err_uart);

			if (port->err_get(&err_uart)) {
				*p_err = AMT21Err::UART_ERROR;
				break;
			}
			byte_ctr++;

			if (err_get(p_err)) {
				break;
			}
		}
		currMillis = millis();
		diffMillis = currMillis - prevMillis;

		if (diffMillis >= READ_TIMOUT_MS) {
			*p_err = AMT21Err::READ_TIMEOUT;
#if defined(AMT21_DEBUG)
			Serial.print("READ_TIMEOUT ");
#endif
			break;
		}
	}
	// Enable Driver-output-enable input
	port->set_halfduplex_pin(1);
}





#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO
