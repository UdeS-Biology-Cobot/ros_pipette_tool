/*
 * pipette_tool_slave_serial.h
 *
 *  Created on: Jun 19, 2018
 *      Author: robotic_tools
 */

#ifndef PIPETTE_TOOL_SLAVE_SERIAL_H_
#define PIPETTE_TOOL_SLAVE_SERIAL_H_


#if defined(ARDUINO)

#include "Arduino.h"
//#include "SoftwareSerial.h"

//#define DRV8880_REGISTER_BASE 0





//#include "protocol_drv8880_conf.h"
#include <robotic_tools/pipette_tool/protocol/pipette_tool_slave_protocol.h>


//#include <pipette_tool_slave_protocol.h>



class PipetteToolSerialSlave
{
public:

	static const int NO_DIRECTION_CONTROL_PIN = -1;


	PipetteToolSerialSlave(PipetteToolSlaveProtocol* protocol, Serial_* serial, uint32_t baud_rate, int direction_control_pin=NO_DIRECTION_CONTROL_PIN);
	PipetteToolSerialSlave(PipetteToolSlaveProtocol* protocol, Serial_* serial, uint32_t baud_rate, uint8_t baud_conf, int direction_control_pin=NO_DIRECTION_CONTROL_PIN);					/* USB Serial already created as "Serial"		*/
	PipetteToolSerialSlave(PipetteToolSlaveProtocol* protocol, HardwareSerial* hw_serial, uint32_t baud_rate, int direction_control_pin=NO_DIRECTION_CONTROL_PIN);
	PipetteToolSerialSlave(PipetteToolSlaveProtocol* protocol, HardwareSerial* serial, uint32_t baud_rate, uint8_t baud_conf, int direction_control_pin=NO_DIRECTION_CONTROL_PIN);			/* Hardware serial already created as "Serial1"	*/
	//PipetteTool(SoftwareSerial* serial);			/* Software serial not created					*/


	void start();

	enum SerialMode {USB_SERIAL, HW_SERIAL, SW_SERIAL};




	static const bool TRANSMIT_LEVEL = HIGH;
	static const bool RECEIVE_LEVEL = LOW;



protected:
	//~PipetteTool();



private:
	//DRV8880* driver;

	PipetteToolSlaveProtocol* protocol;
	Serial_* 		 usb_serial = nullptr;
	HardwareSerial*  hw_serial = nullptr;
	//SoftwareSerial*  sw_serial = nullptr;

	SerialMode mode;
	unsigned long int baud_rate;
	unsigned char baud_conf;

	void usb_loop();
	void hw_loop();

	int direction_control_pin;



};
#endif

#endif /* PIPETTE_TOOL_SLAVE_SERIAL_H_ */
