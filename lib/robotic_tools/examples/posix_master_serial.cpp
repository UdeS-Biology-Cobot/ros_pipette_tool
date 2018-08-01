/*
 * main.cpp
 *
 *  Created on: Jun 19, 2018
 *      Author: biobot
 */
/*
#define htons(x) ( ((x)<< 8 & 0xFF00) | \
                   ((x)>> 8 & 0x00FF) )
#define ntohs(x) htons(x)

#define htonl(x) ( ((x)<<24 & 0xFF000000UL) | \
                   ((x)<< 8 & 0x00FF0000UL) | \
                   ((x)>> 8 & 0x0000FF00UL) | \
                   ((x)>>24 & 0x000000FFUL) )
#define ntohl(x) htonl(x)
*/

//#include <biobot/pipette_tool/posix/pipette_tool_serial_master.h>

/* TODO remove this define HACK*/

/*
#include <biobot/pipette_tool/posix/pipette_tool_serial_master.h>
*/
#include <biobot/error_handle/error_handle.h>
#include <biobot/pipette_tool/posix/pipette_tool_serial_master.h>
#include <biobot/pipette_tool/protocol/pipette_tool_master_protocol.h>


ErrorHandle err_handle = ErrorHandle();
ErrorHandle* p_err = &err_handle;

#define SLAVE_ADDR 0x01

//#include <boost/asio/serial_port.hpp>
//#include <boost/asio.hpp>

//using namespace boost;
#include <boost/bind.hpp>


int main (void) {

	asio::io_service io;
	asio::serial_port port(io);

	std::string device = "/dev/pipette_tool";

	port.open(device);
	port.set_option(asio::serial_port_base::baud_rate(1000000));

	PipetteToolSerialMaster serial = PipetteToolSerialMaster(&port);

	boost::function<uint8_t(uint8_t*, uint8_t)> cb_write;
	boost::function<uint8_t()> cb_read;

	cb_write = boost::bind(&PipetteToolSerialMaster::write, &serial, _1, _2);
	cb_read = boost::bind(&PipetteToolSerialMaster::read, &serial);

	PipetteToolMasterProtocol protocol = PipetteToolMasterProtocol(SLAVE_ADDR, cb_write, cb_read);



	uint32_t params[16] = {0};
	uint32_t f_min = 500;
	uint32_t f_max = 4800;
	uint32_t slope = 100;

	params[1] = f_min;
	params[2] = f_max;
	params[3] = slope;


	protocol.cmd_move(PipetteTool::Direction::ASPIRATE, 300.0, Trajectory::Profile::CONCAVE_SCURVE_VEL, params);
	protocol.cmd_move(PipetteTool::Direction::DISPENSE, 300.0, Trajectory::Profile::CONCAVE_SCURVE_VEL, params);
	//protocol.cmd_move(PipetteTool::Direction::DISPENSE, 50.0, Trajectory::Profile::CONCAVE_SCURVE_VEL, params);
}
