/*
 * pipette_tool_slave_serial.cpp
 *
 *  Created on: Jun 19, 2018
 *      Author: robotic_tools
 */




/*
 * pipette_tool.cpp
 *
 *  Created on: May 24, 2018
 *      Author: robotic_tools
 */

//#include "pipette_tool_slave_serial.h"
#if defined(ARDUINO)

#include <robotic_tools/pipette_tool/arduino/pipette_tool_slave_serial.h>

PipetteToolSerialSlave::PipetteToolSerialSlave(PipetteToolSlaveProtocol* protocol, Serial_* usb_serial, uint32_t baud_rate, int direction_control_pin)
:protocol(protocol), usb_serial(usb_serial), mode(USB_SERIAL), baud_rate(baud_rate), baud_conf(SERIAL_8N1), direction_control_pin(direction_control_pin)
{}

PipetteToolSerialSlave::PipetteToolSerialSlave(PipetteToolSlaveProtocol* protocol, Serial_* usb_serial, uint32_t baud_rate, uint8_t baud_conf, int direction_control_pin)
:protocol(protocol), usb_serial(usb_serial), mode(USB_SERIAL), baud_rate(baud_rate), baud_conf(baud_conf), direction_control_pin(direction_control_pin)
{}


PipetteToolSerialSlave::PipetteToolSerialSlave(PipetteToolSlaveProtocol* protocol, HardwareSerial* hw_serial, uint32_t baud_rate, int direction_control_pin)
:protocol(protocol), hw_serial(hw_serial), mode(HW_SERIAL), baud_rate(baud_rate), baud_conf(SERIAL_8N1), direction_control_pin(direction_control_pin)
{}

PipetteToolSerialSlave::PipetteToolSerialSlave(PipetteToolSlaveProtocol* protocol, HardwareSerial* hw_serial, uint32_t baud_rate, uint8_t baud_conf, int direction_control_pin)
:protocol(protocol), hw_serial(hw_serial), mode(HW_SERIAL), baud_rate(baud_rate), baud_conf(baud_conf), direction_control_pin(direction_control_pin)
{}

/*
PipetteTool::PipetteTool(SoftwareSerial* sw_serial)
:sw_serial(sw_serial), mode(SW_SERIAL)
{}
*/





//static const uint8_t DRIVER_REGISTER_BASE = 0x00;







void PipetteToolSerialSlave::start()
{

	if (direction_control_pin != NO_DIRECTION_CONTROL_PIN) {
		pinMode(9, OUTPUT);
	}


	if (this->mode == USB_SERIAL) {
		usb_serial->begin(baud_rate, baud_conf);
		usb_loop();
	}
	else if (this->mode == HW_SERIAL) {
		hw_serial->begin(baud_rate, baud_conf);
		hw_loop();
	}
}



void PipetteToolSerialSlave::usb_loop() {

	int bytesSent = 0;
	byte b_read;

	byte* datagram = nullptr;

	/* TODO add timeout ? */

	while (true) {


		if (direction_control_pin != NO_DIRECTION_CONTROL_PIN) {
			digitalWrite(direction_control_pin, RECEIVE_LEVEL);
		}

		while (datagram == nullptr)
		{
			if (usb_serial->available() > 0) {
				b_read = usb_serial->read();
				datagram = protocol->read_command(b_read);
			}
		}


		if (direction_control_pin != NO_DIRECTION_CONTROL_PIN) {
			digitalWrite(direction_control_pin, TRANSMIT_LEVEL);
		}

		bytesSent = 0;

		while (bytesSent != protocol->DATAGRAM_SIZE) {
			bytesSent += usb_serial->write(datagram, protocol->DATAGRAM_SIZE);
		}

		datagram = nullptr;
	}
}



void PipetteToolSerialSlave::hw_loop() {

	int bytesSent = 0;
	byte b_read;

	byte* datagram = nullptr;

	/* TODO add timeout ? */

	while (true) {


		if (direction_control_pin != NO_DIRECTION_CONTROL_PIN) {
			digitalWrite(direction_control_pin, RECEIVE_LEVEL);
		}

		while (datagram == nullptr)
		{
			if (hw_serial->available() > 0) {
				b_read = hw_serial->read();
				datagram = protocol->read_command(b_read);
			}
		}


		if (direction_control_pin != NO_DIRECTION_CONTROL_PIN) {
			digitalWrite(direction_control_pin, TRANSMIT_LEVEL);
		}

		bytesSent = 0;

		while (bytesSent != protocol->DATAGRAM_SIZE) {
			bytesSent += hw_serial->write(datagram, protocol->DATAGRAM_SIZE);
		}

		datagram = nullptr;
	}
}


#endif





