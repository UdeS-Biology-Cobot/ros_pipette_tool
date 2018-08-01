/*
 * pipette_tool_serial_master.h
 *
 *  Created on: Jun 19, 2018
 *      Author: robotic_tools
 */

#ifndef PIPETTE_TOOL_MASTER_SERIAL_H_
#define PIPETTE_TOOL_MASTER_SERIAL_H_

#include <robotic_tools/pipette_tool/protocol/pipette_tool_master_protocol.h>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

using namespace boost;

class PipetteToolSerialMaster
{
public:

	PipetteToolSerialMaster(asio::serial_port*);

	uint8_t read();
	uint8_t write(uint8_t* buf, uint8_t buf_size);



private:
	//PipetteToolMasterProtocol protocol = PipetteToolMasterProtocol(0x01);
	boost::system::error_code ec;

	asio::serial_port* port;


public:


};


#endif /* PIPETTE_TOOL_MASTER_SERIAL_H_ */
