/*
 * uart_serial.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: biobot
 */

#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)


#include <robotic_tool/uart/arduino/uart_serial.h>
#include "wiring_private.h" // pinPeripheral() function



UartSerial::UartSerial(SERCOM* _s,
					   Sercom* sercom,
					  uint8_t  _pinRX,
					  uint8_t  _pinTX,
				  SercomRXPad  _padRX,
			  SercomUartTXPad  _padTX,
			  UartSerialErr *p_err)
: Uart(_s, _pinRX, _pinTX, _padRX, _padTX),
  sercom(sercom),
  rx_pad(_padRX),
  tx_pad(_padTX),
  pin_hd(0),
  mode_hd(nullptr),
  out_hd(nullptr),
  pin_rx(_pinRX),
  pin_tx(_pinTX),
  HALF_DUPLEX_MODE(false)
{
	// Check if pins are valid
	validate_pin(pin_rx, p_err);
	if (err_get(p_err)) {return;}

	validate_pin(pin_tx, p_err);
	if (err_get(p_err)) {return;}

	// Check if pins number are neighbors
	if (abs((int)pin_rx - (int)pin_tx) != 1) {
		*p_err = UartSerialErr::INVALID_PIN;
		return;
	}

	validate_sercom(p_err);
	if (err_get(p_err)) {return;}

	// TODO add mappin check
	validate_mapping(p_err);
	if (err_get(p_err)) {return;}
}


UartSerial::UartSerial(SERCOM* _s,
					   Sercom* sercom,
					  uint8_t  _pinRX,
					  uint8_t  _pinTX,
				  SercomRXPad  _padRX,
			  SercomUartTXPad  _padTX,
			  	  	 uint32_t  _pinHD,
					 UartSerialErr *p_err)
: Uart(_s, _pinRX, _pinTX, _padRX, _padTX),
  sercom(sercom),
  rx_pad(_padRX),
  tx_pad(_padTX),
  pin_hd(digitalPinToBitMask(_pinHD)),
  mode_hd(portModeRegister(digitalPinToPort(_pinHD))),
  out_hd(portOutputRegister(digitalPinToPort(_pinHD))),
  pin_rx(_pinRX),
  pin_tx(_pinTX),
  HALF_DUPLEX_MODE(true)
{
	// Initialize pin
	*mode_hd |= pin_hd;
	*out_hd &= ~pin_hd;

	// Check if pins are valid
	validate_pin(pin_rx, p_err);
	if (err_get(p_err)) {return;}

	validate_pin(pin_tx, p_err);
	if (err_get(p_err)) {return;}

	// Check if pins number are neighbors
	if (abs((int)pin_rx - (int)pin_tx) != 1) {
		*p_err = UartSerialErr::INVALID_PIN;
		return;
	}

	validate_sercom(p_err);
	if (err_get(p_err)) {return;}

	// TODO add mappin check
	validate_mapping(p_err);
	if (err_get(p_err)) {return;}


}




void UartSerial::validate_pin(uint8_t pin, UartSerialErr *p_err)
{
	for (uint32_t i = 0; i < AVAILABLE_PIN_SIZE; i++)
	{
		if (pin == available_pin[i]) {
			return;
		}
	}

	*p_err = UartSerialErr::INVALID_PIN;
}


void UartSerial::validate_sercom(UartSerialErr *p_err)
{

	for (uint32_t i = 0; i < SERCOM_INST_NUM; i++)
	{
		if (sercom == sercom_list[i]) {
			return;
		}
	}

	*p_err = UartSerialErr::INVALID_SERCOM;
}


void UartSerial::validate_mapping(UartSerialErr *p_err)
{

	if (pin_tx == 0 && pin_rx == 1){

		if (tx_pad == SercomUartTXPad::UART_TX_PAD_0 && rx_pad == SercomRXPad::SERCOM_RX_PAD_1) {

			if (sercom == SERCOM3) {
				pinPeripheral(pin_rx, PIO_SERCOM);
				pinPeripheral(pin_tx, PIO_SERCOM);
				return;
			}
			else if (sercom == SERCOM5) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 2 && pin_rx == 3) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_2 && rx_pad == SercomRXPad::SERCOM_RX_PAD_3) {

			if (sercom == SERCOM0) {
				pinPeripheral(pin_rx, PIO_SERCOM);
				pinPeripheral(pin_tx, PIO_SERCOM);
				return;
			}
			else if (sercom == SERCOM2) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 4 && pin_rx == 5) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_2 && rx_pad == SercomRXPad::SERCOM_RX_PAD_3) {

			if (sercom == SERCOM4) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 6 && pin_rx == 7) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_2 && rx_pad == SercomRXPad::SERCOM_RX_PAD_3) {

			if (sercom == SERCOM5) {
				pinPeripheral(pin_rx, PIO_SERCOM);
				pinPeripheral(pin_tx, PIO_SERCOM);
				return;
			}
			else if (sercom == SERCOM3) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 8 && (pin_rx == 9 || pin_rx == 10)) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_0 && (rx_pad == SercomRXPad::SERCOM_RX_PAD_1 || rx_pad == SercomRXPad::SERCOM_RX_PAD_3)) {

			if (sercom == SERCOM1) {
				pinPeripheral(pin_rx, PIO_SERCOM);
				pinPeripheral(pin_tx, PIO_SERCOM);
				return;
			}
			else if (sercom == SERCOM3) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 11 && pin_rx == 12) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_0 && rx_pad == SercomRXPad::SERCOM_RX_PAD_1) {

			if (sercom == SERCOM0) {
				pinPeripheral(pin_rx, PIO_SERCOM);
				pinPeripheral(pin_tx, PIO_SERCOM);
				return;
			}
			else if (sercom == SERCOM2) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 14 && pin_rx == 13) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_2 && rx_pad == SercomRXPad::SERCOM_RX_PAD_3) {

			if (sercom == SERCOM5) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 16 && pin_rx == 17) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_0 && rx_pad == SercomRXPad::SERCOM_RX_PAD_1) {

			if (sercom == SERCOM5) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 18 && pin_rx == 19) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_0 && rx_pad == SercomRXPad::SERCOM_RX_PAD_1) {

			if (sercom == SERCOM0) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 20 && pin_rx == 21) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_2 && rx_pad == SercomRXPad::SERCOM_RX_PAD_3) {

			if (sercom == SERCOM0) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 22 && pin_rx == 23) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_2 && rx_pad == SercomRXPad::SERCOM_RX_PAD_3) {

			if (sercom == SERCOM3) {
				pinPeripheral(pin_rx, PIO_SERCOM);
				pinPeripheral(pin_tx, PIO_SERCOM);
				return;
			}
			else if (sercom == SERCOM5) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 26 && pin_rx == 27) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_0 && rx_pad == SercomRXPad::SERCOM_RX_PAD_1) {

			if (sercom == SERCOM2) {
				pinPeripheral(pin_rx, PIO_SERCOM);
				pinPeripheral(pin_tx, PIO_SERCOM);
				return;
			}
			else if (sercom == SERCOM4) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 28 && pin_rx == 29) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_2 && rx_pad == SercomRXPad::SERCOM_RX_PAD_3) {

			if (sercom == SERCOM2) {
				pinPeripheral(pin_rx, PIO_SERCOM);
				pinPeripheral(pin_tx, PIO_SERCOM);
				return;
			}
			else if (sercom == SERCOM4) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 32 && pin_rx == 33) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_0 && rx_pad == SercomRXPad::SERCOM_RX_PAD_1) {

			if (sercom == SERCOM4) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}
	else if (pin_tx == 34 && pin_rx == 35) {
		if (tx_pad == SercomUartTXPad::UART_TX_PAD_0 && rx_pad == SercomRXPad::SERCOM_RX_PAD_1) {

			if (sercom == SERCOM1) {
				pinPeripheral(pin_rx, PIO_SERCOM_ALT);
				pinPeripheral(pin_tx, PIO_SERCOM_ALT);
				return;
			}
		}
	}


	*p_err = UartSerialErr::INVALID_PIN_SERCOM_MAPPING;


}







/*
void UartSerial::uart_pin_init()
{
	pinPeripheral(pin_rx, PIO_SERCOM_ALT);
	pinPeripheral(pin_tx, PIO_SERCOM_ALT);
}

*/















int UartSerial::read(UartSerialErr* p_err)
{
	int byte = Uart::read();

	if (sercom->USART.STATUS.bit.FERR) {
		*p_err = UartSerialErr::UART_FRAME_ERROR;
		sercom->USART.STATUS.bit.FERR = 1;		// Clear error
	}

	// Buffer Overflow detected
	if (sercom->USART.STATUS.bit.BUFOVF) {
		*p_err = UartSerialErr::UART_BUFOVF;
		sercom->USART.STATUS.bit.BUFOVF = 1;	// Clear error
	}

	// Frame error detected
	if (sercom->USART.STATUS.bit.PERR) {
		*p_err = UartSerialErr::UART_PARITY_ERROR;
		sercom->USART.STATUS.bit.PERR = 1;		// Clear error
	}

	if (byte < 0) {
		*p_err =  UartSerialErr::UART_BUFEMPTY;
	}


	return byte;
}

void UartSerial::flush_rx()
{
	while (Uart::available() > 0){
		Uart::read();
	};
}

void UartSerial::flush_tx()
{
	while(!sercom->USART.INTFLAG.bit.TXC);
}

void UartSerial::set_halfduplex_pin(bool pin_state)
{
	if (HALF_DUPLEX_MODE) {
		if (pin_state) {
			*out_hd |= pin_hd;
		} else {
			*out_hd &= ~pin_hd;
		}
	}
}



void UartSerial::begin(unsigned long baudRate)
{
	Uart::begin(baudRate);
}


void UartSerial::begin(unsigned long baudrate, uint16_t config)
{
	Uart::begin(baudrate, config);
}


void UartSerial::end()
{
	Uart::end();
}


int UartSerial::available()
{
	return Uart::available();
}


int UartSerial::availableForWrite()
{
	return Uart::availableForWrite();
}


int UartSerial::peek()
{
	return Uart::peek();
}




size_t UartSerial::write(const uint8_t data)
{
	return Uart::write(data);
}




void UartSerial::IrqHandler()
{
	Uart::IrqHandler();
}



size_t UartSerial::write(const uint8_t *buffer, size_t size)
{
	return Uart::write(buffer, size);
}



bool UartSerial::err_get(UartSerialErr* p_err)
{
	if (*p_err == UartSerialErr::NONE) {
		return false;
	}
	return true;
}

void UartSerial::err_clear(UartSerialErr* p_err)
{
	*p_err = UartSerialErr::NONE;
}




#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO










