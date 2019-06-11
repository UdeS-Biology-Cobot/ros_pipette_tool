/*
 * drv8880.h
 *
 *  Created on: May 23, 2018
 *      Author: biobot
 */

#ifndef TMC2208_H_
#define TMC2208_H_


#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)

//#include <stdint.h>
//#include <Arduino.h>
//#include <Uart.h>
#include <robotic_tool/uart/arduino/uart_serial.h>
#include <robotic_tool/driver/tmc2208_base.h>


// For debugging purposes on SerialUSB
//#define TMC2208_DEBUG


class TMC2208 : public TMC2208Base
{
public:

	TMC2208(UartSerial* port, uint32_t baudrate, uint32_t _pin_dir, uint32_t _pin_en, TMC2208Err* p_err);
	// GCONF




private:

	// Override functions
	void delay_ms(uint32_t) override;

	void set_enable_pin(LogicLevel) override;
	void set_direction_pin(LogicLevel) override;
	void set_halfduplex_pin(LogicLevel) override;

	void serial_flush_rx() override;
	void serial_flush_tx() override;
	void serial_write(const uint8_t *buffer, uint32_t size) override;
	void serial_read(uint8_t *buffer, uint32_t size, TMC2208Err* p_err) override;



	//Sercom* get_sercom(SercomPort sercom_port, TMC2208Err* p_err);

	void init_pin();

	UartSerial* port;
	//Sercom* sercom;
	uint32_t baudrate;


	// pin
	//uint32_t pin_half_duplex;
	//volatile uint32_t *mode_half_duplex;
	//volatile uint32_t *out_half_duplex;

	uint32_t pin_dir;
	volatile uint32_t *mode_dir;
	volatile uint32_t *out_dir;

	uint32_t pin_en;
	volatile uint32_t *mode_en;
	volatile uint32_t *out_en;


};


#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO

#endif /* TMC2208_H_ */
