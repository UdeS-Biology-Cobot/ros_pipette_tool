/*
 * pipette_tool_serial_master.cpp
 *
 *  Created on: Jun 19, 2018
 *      Author: robotic_tools
 */




#include <robotic_tools/pipette_tool/posix/pipette_tool_serial_master.h>


PipetteToolSerialMaster::PipetteToolSerialMaster(asio::serial_port* port)
:port(port)
{}

uint8_t PipetteToolSerialMaster::read()
{
	uint8_t byte;
	bool byte_found = false;

	while (!byte_found) {
		port->read_some(asio::buffer(&byte, 1), ec);

		if (!ec) {
			byte_found = true;
		}
	}


	return byte;
}

uint8_t PipetteToolSerialMaster::write(uint8_t* buf, uint8_t buf_size)
{
	return port->write_some(boost::asio::buffer(buf, buf_size));
}


