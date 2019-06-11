/*
 * drv8880.cpp
 *
 *  Created on: May 23, 2018
 *      Author: biobot
 */

#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)


#include <robotic_tool/driver/arduino/tmc2208.h>
#include <Arduino.h>




TMC2208::TMC2208(UartSerial* port,
		     uint32_t  baudrate,
			 uint32_t  _pin_dir,
			 uint32_t  _pin_en,
		   TMC2208Err* p_err)
: TMC2208Base(baudrate),
  port(port),
  //sercom(get_sercom(sercom_port, p_err)),
  baudrate(baudrate),
  //pin_half_duplex(digitalPinToBitMask(half_duplex_pin)),
  //mode_half_duplex(portModeRegister(digitalPinToPort(half_duplex_pin))),
  //out_half_duplex(portOutputRegister(digitalPinToPort(half_duplex_pin))),

  pin_dir(digitalPinToBitMask(_pin_dir)),
  mode_dir(portModeRegister(digitalPinToPort(_pin_dir))),
  out_dir(portOutputRegister(digitalPinToPort(_pin_dir))),

  pin_en(digitalPinToBitMask(_pin_en)),
  mode_en(portModeRegister(digitalPinToPort(_pin_en))),
  out_en(portOutputRegister(digitalPinToPort(_pin_en)))
{
	port->begin(baudrate);
	delayMicroseconds(500);	// Corresponds to low baudrate + RC filter (5 Tau -> 1k_ohm + Capacitor)

	init_pin();
	enable_motor();

	// Fill all registers data
	read_all_registers(p_err);
	if (err_get(p_err)) {goto error;}

	// IMPORTANT order of commands is critical

	// Set to spreadCycle mode
	set_en_spread_cycle(EN_SPREAD_CYCLE::STEALTH_CHOP_ENABLED, p_err);
	if (err_get(p_err)) {goto error;}

	// Enable micro step selection by uart register
	set_mstep_reg_select(MSTEP_REG_SELECT::REGISTER, p_err);
	if (err_get(p_err)) {goto error;}

	// Disable standbuy current control (hardware)
	set_pdn_disable(PDN_DISABLE::DISABLE_STANDSTILL_CURRENT_CONTROL_VIA_PDN_UART_PIN, p_err);
	if (err_get(p_err)) {goto error;}

	// Set Default toff
	set_toff(3, p_err);
	if (err_get(p_err)) {goto error;}

	// Reset GSTAT
	write_register(&GSTAT, 0xFFFFFFFF, p_err);
	if (err_get(p_err)) {return;}

	return;

error:
 	 disable_motor();
}




void TMC2208::serial_flush_rx()
{
	port->flush_rx();
}

void TMC2208::serial_flush_tx()
{
	port->flush_tx();
}



void TMC2208::delay_ms(uint32_t _delay)
{
	delay(_delay);
}

void TMC2208::set_enable_pin(LogicLevel pin_state)
{
	if (pin_state == LogicLevel::LOW_LEVEL) {
		*out_en &= ~pin_en;
	} else {
		*out_en |= pin_en;
	}
}

void TMC2208::set_direction_pin(LogicLevel pin_state)
{
	if (pin_state == LogicLevel::LOW_LEVEL) {
		*out_dir &= ~pin_dir;
	} else {
		*out_dir |= pin_dir;
	}
}

void TMC2208::set_halfduplex_pin(LogicLevel pin_state)
{
	port->set_halfduplex_pin((bool)pin_state);
}









void TMC2208::init_pin()
{
	port->set_halfduplex_pin(1);	// // 1 = TX enable, 0 = RX enable

	*mode_dir |= pin_dir;
	*out_dir &= ~pin_dir;	//

	*mode_en |= pin_en;
	*out_en &= ~pin_en;	// 0 = enable, 1 = disable
}








// Get register data
void TMC2208::serial_read(uint8_t *buffer, uint32_t size, TMC2208Err* p_err)
{
	uint8_t byte_ctr;
	//uint8_t crc;
	uint32_t prevMillis;
	uint32_t currMillis;
	uint32_t diffMillis;

	// Read from serial
	byte_ctr = 0;
	prevMillis = millis();

	UartSerialErr err_uart = UartSerialErr::NONE;

	while (byte_ctr != READ_ACCESS_REPLY_BUF_LEN) {
		if (port->available() > 0) {
			buffer[byte_ctr] = port->read(&err_uart);

			if (port->err_get(&err_uart)) {
				*p_err = TMC2208Err::UART_ERROR;
				break;
			}

			byte_ctr++;

			if (*p_err != TMC2208Err::NONE) {
				break;
			}
		}

		currMillis = millis();
		diffMillis = currMillis - prevMillis;

		if (diffMillis >= READ_TIMOUT_MS) {
			*p_err = TMC2208Err::READ_TIMEOUT;
#if defined(TMC2208_DEBUG)
			Serial.print("READ_TIMEOUT ");
#endif
			break;
		}
	}
}


void TMC2208::serial_write(const uint8_t *buffer, uint32_t buf_size)
{
	uint8_t byte_ctr = 0;
	while (byte_ctr != buf_size) {
		byte_ctr += port->write(tx_buf, buf_size);
	}
}
















#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO




