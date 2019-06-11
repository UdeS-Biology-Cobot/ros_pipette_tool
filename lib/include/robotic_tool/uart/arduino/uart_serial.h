/*
 * uart_serial.h
 *
 *  Created on: Dec 5, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_UART_ARDUINO_UART_SERIAL_H_
#define LIB_BIOBOT_BIOBOT_UART_ARDUINO_UART_SERIAL_H_



#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)

#include <stdint.h>
#include <Arduino.h>
#include <Uart.h>



enum class UartSerialErr : uint8_t {
	NONE,


	INVALID_PIN,
	INVALID_SERCOM,
	INVALID_PIN_SERCOM_MAPPING,

	UART_BUFOVF,
	UART_FRAME_ERROR,
	UART_PARITY_ERROR,
	UART_BUFEMPTY,
};




class UartSerial : private Uart
{
public:

	static const uint8_t AVAILABLE_PIN_SIZE = 34;
	uint8_t available_pin[AVAILABLE_PIN_SIZE] = {
			0,
			1,
			2,
			3,
			4,
			5,
			6,
			7,
			8,
			9,
			10,
			11,
			12,
			13,
			14,
			// 15 is not available
			16,
			17,
			18,
			19,
			20,
			21,
			22,
			23,
			// 24 is listed alone so not sure...
			//25 is not available
			26,
			27,
			28,
			29,
			// 30 is not available
			// 31 is not available
			32,
			33,
			34,
			35
	};


	Sercom* sercom_list[SERCOM_INST_NUM] = SERCOM_INSTS;



	UartSerial(SERCOM *_s, Sercom* sercom, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX, UartSerialErr *p_err);
	UartSerial(SERCOM *_s, Sercom* sercom, uint8_t _pinRX, uint8_t _pinTX, SercomRXPad _padRX, SercomUartTXPad _padTX, uint32_t _pinHD, UartSerialErr *p_err);

	virtual ~UartSerial() {};

	// Public class from Uart
    void begin(unsigned long baudRate);
    void begin(unsigned long baudrate, uint16_t config);
    void end();
    int available();
    int availableForWrite();
    int peek();


    size_t write(const uint8_t data);
    using Print::write; // pull in write(str) and write(buf, size) from Print

    void IrqHandler();

    operator bool() { return true; }

    size_t write(const uint8_t *buffer, size_t size);



    // New and modified classes



	int read(UartSerialErr* p_err);	//override Uart read to be able to check errors
	void flush_rx();
	void flush_tx();
	void set_halfduplex_pin(bool pin_state);


	bool err_get(UartSerialErr* p_err);
	void err_clear(UartSerialErr* p_err);


	void validate_pin(uint8_t pin, UartSerialErr *p_err);
	void validate_sercom(UartSerialErr *p_err);
	void validate_mapping(UartSerialErr *p_err);

private:
	Sercom *sercom;

	// Half duplex pin
	SercomRXPad rx_pad;
	SercomUartTXPad tx_pad;

	uint32_t pin_hd;	// half-duplex
	volatile uint32_t *mode_hd;
	volatile uint32_t *out_hd;

	// Serial Rx and Tx pin
	uint8_t pin_rx;
	uint8_t pin_tx;


	//void uart_pin_init();

	const bool HALF_DUPLEX_MODE;
};




#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO




#endif /* LIB_BIOBOT_BIOBOT_UART_ARDUINO_UART_SERIAL_H_ */
