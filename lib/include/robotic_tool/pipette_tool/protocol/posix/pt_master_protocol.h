/*
 * pt_master_protocol.h
 *
 *  Created on: Dec 10, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_PROTOCOL_POSIX_PT_MASTER_PROTOCOL_H_
#define LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_PROTOCOL_POSIX_PT_MASTER_PROTOCOL_H_

#if defined(__linux__)

#include <robotic_tool/pipette_tool/protocol/pt_protocol_base.h>
//#include <biobot/pipette_tool/pipette_tool.h>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>
#include <robotic_tool/util/endian/endian.h>

// using namespace boost;

class PtMasterProtocol : public PtProtocolBase
{
public:
	PtMasterProtocol(boost::asio::serial_port* port, uint32_t baudrate, const std::string device);

	void aspirate(uint32_t nl, double speed, PtProtocolErr* p_err);
	void dispense(uint32_t nl, double speed, PtProtocolErr* p_err);

	void enable_motor(bool enable_flag, PtProtocolErr* p_err);
	void multiple_sequence(uint32_t nl, double speed, Direction start_dir, uint32_t seq_number, PtProtocolErr* p_err);

	void do_homing(PtProtocolErr* p_err);
	void eject_tip(PtProtocolErr* p_err);

	void move_to_top(PtProtocolErr* p_err);
	void move_to_tip(PtProtocolErr* p_err);

	uint32_t get_max_nl(PtProtocolErr* p_err);

	double get_max_speed(PtProtocolErr* p_err);
	int32_t get_offset_nl(PtProtocolErr* p_err);

	uint32_t get_rem_aspirate_vol_nl(PtProtocolErr* p_err);
	uint32_t get_rem_dispense_vol_nl(PtProtocolErr* p_err);

	uint32_t get_serial_number(PtProtocolErr* p_err);

	/*
	ASPIRATE,
	DISPENSE,
	MULTI_SEQUENCE,
	HOMING,
	EJECT_TIP,
	MAX_NL,
	SERIAL_NUMBER,
	*/

private:
	boost::asio::serial_port* port;

	boost::system::error_code ec;

	void store_data_to_buf(uint8_t _tx_buf[], uint8_t data_buf[], uint8_t buf_len);

	void read(uint8_t _rx_buf[], uint8_t data_buf[], PtProtocolErr* p_err);

	uint8_t tx_buf[64];
	uint8_t rx_buf[64];

	const uint8_t SLAVE_FRAME = 0x00;
};

#endif  // __linux__

#endif /* LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_PROTOCOL_POSIX_PT_MASTER_PROTOCOL_H_ */
