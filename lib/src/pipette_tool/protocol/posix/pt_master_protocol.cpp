/*
 * pt_master_protocol.cpp
 *
 *  Created on: Dec 10, 2018
 *      Author: biobot
 */

#if defined(__linux__)

#include <robotic_tool/pipette_tool/protocol/posix/pt_master_protocol.h>
#include <iostream>

PtMasterProtocol::PtMasterProtocol(boost::asio::serial_port* port, uint32_t baudrate, const std::string device)
  : port(port) {
	port->open(device);
	port->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
}

void PtMasterProtocol::aspirate(uint32_t nl, double speed, PtProtocolErr* p_err) {
	uint8_t* data_buf;
	uint8_t rx_data_buf[8];
	boost::system::error_code ec_boost;

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5 + 2 + sizeof(nl) + sizeof(speed);

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::ASPIRATE;

	// Store nano liters
	tx_buf[4] = (uint8_t)DataRegIdx::NANO_LITERS;
	data_buf = (uint8_t*)&nl;
	store_data_to_buf(&tx_buf[5], data_buf, sizeof(nl));

	// Store speed
	tx_buf[9] = (uint8_t)DataRegIdx::SPEED;
	data_buf = (uint8_t*)&speed;
	store_data_to_buf(&tx_buf[10], data_buf, sizeof(speed));

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			std::cout << "aspirtate transmit error" << std::endl;
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return;
		}
	}

	// Read response
	read(rx_buf, rx_data_buf, p_err);
	if (err_get(p_err)) {
		return;
	}
}

void PtMasterProtocol::dispense(uint32_t nl, double speed, PtProtocolErr* p_err) {
	uint8_t* data_buf;
	uint8_t rx_data_buf[8];
	boost::system::error_code ec_boost;

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5 + 2 + sizeof(nl) + sizeof(speed);

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::DISPENSE;

	// Store nano liters
	tx_buf[4] = (uint8_t)DataRegIdx::NANO_LITERS;
	data_buf = (uint8_t*)&nl;
	store_data_to_buf(&tx_buf[5], data_buf, sizeof(nl));

	// Store speed
	tx_buf[9] = (uint8_t)DataRegIdx::SPEED;
	data_buf = (uint8_t*)&speed;
	store_data_to_buf(&tx_buf[10], data_buf, sizeof(speed));

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			std::cout << "Error Transmit error3" << std::endl;
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return;
		}
	}

	// Read response
	read(rx_buf, rx_data_buf, p_err);
	if (err_get(p_err)) {
		return;
	}
}

void PtMasterProtocol::enable_motor(bool enable_flag, PtProtocolErr* p_err) {
	uint8_t* data_buf;
	uint8_t rx_data_buf[8];
	boost::system::error_code ec_boost;

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5 + 1 + sizeof(enable_flag);

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::ENABLE_MOTOR;

	// Store nano liters
	tx_buf[4] = (uint8_t)DataRegIdx::MOTOR_STATE;
	data_buf = (uint8_t*)&enable_flag;
	store_data_to_buf(&tx_buf[5], data_buf, sizeof(enable_flag));

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			std::cout << "Error Transmit error3" << std::endl;
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return;
		}
	}

	// Read response
	read(rx_buf, rx_data_buf, p_err);
	if (err_get(p_err)) {
		return;
	}
}

void PtMasterProtocol::multiple_sequence(uint32_t nl, double speed, Direction start_dir, uint32_t seq_number,
                                         PtProtocolErr* p_err) {
	uint8_t* data_buf;
	uint8_t rx_data_buf[8];
	boost::system::error_code ec_boost;

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5 + 4 + sizeof(nl) + sizeof(speed) + sizeof(start_dir) + sizeof(seq_number);

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::MULTI_SEQUENCE;

	// Store nano liters
	tx_buf[4] = (uint8_t)DataRegIdx::NANO_LITERS;
	data_buf = (uint8_t*)&nl;
	store_data_to_buf(&tx_buf[5], data_buf, sizeof(nl));

	// Store speed
	tx_buf[9] = (uint8_t)DataRegIdx::SPEED;
	data_buf = (uint8_t*)&speed;
	store_data_to_buf(&tx_buf[10], data_buf, sizeof(speed));

	// Store direction
	tx_buf[18] = (uint8_t)DataRegIdx::DIRECTION;
	data_buf = (uint8_t*)&start_dir;
	store_data_to_buf(&tx_buf[19], data_buf, sizeof(start_dir));

	// Store sequence number
	tx_buf[20] = (uint8_t)DataRegIdx::SEQUENCE_NUMBER;
	data_buf = (uint8_t*)&seq_number;
	store_data_to_buf(&tx_buf[21], data_buf, sizeof(seq_number));

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			std::cout << "Error Transmit error3" << std::endl;
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return;
		}
	}

	// Read response
	read(rx_buf, rx_data_buf, p_err);
	if (err_get(p_err)) {
		return;
	}
}

void PtMasterProtocol::do_homing(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::HOMING;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return;
	}
}

void PtMasterProtocol::eject_tip(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::EJECT_TIP;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return;
	}
}

uint32_t PtMasterProtocol::get_max_nl(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::MAX_NL;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return 0;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return 0;
	}

	uint32_t nl;
	memcpy(&nl, data_buf, sizeof(nl));
	return nl;
}

uint32_t PtMasterProtocol::get_rem_aspirate_vol_nl(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::REM_ASPIRATE_NL;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			std::cout << "Error Transmit error2" << std::endl;
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return 0;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return 0;
	}

	uint32_t nl;
	memcpy(&nl, data_buf, sizeof(nl));
	return nl;
}

uint32_t PtMasterProtocol::get_rem_dispense_vol_nl(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::REM_DISPENSE_NL;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			std::cout << "Error Transmit error" << std::endl;
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return 0;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return 0;
	}

	uint32_t nl;
	memcpy(&nl, data_buf, sizeof(nl));
	return nl;
}

uint32_t PtMasterProtocol::get_serial_number(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::SERIAL_NUMBER;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return 0;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return 0;
	}

	uint32_t sn;
	memcpy(&sn, data_buf, sizeof(sn));
	return sn;
}

double PtMasterProtocol::get_max_speed(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::MAX_SPEED;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return 0;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return 0;
	}

	double speed;
	memcpy(&speed, data_buf, sizeof(speed));
	return speed;
}

int32_t PtMasterProtocol::get_offset_nl(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::OFFSET_NL;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return 0;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return 0;
	}

	int32_t nl;
	memcpy(&nl, data_buf, sizeof(nl));
	return nl;
}

void PtMasterProtocol::move_to_top(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::MOVE_TO_TOP;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return;
	}
}

void PtMasterProtocol::move_to_tip(PtProtocolErr* p_err) {
	boost::system::error_code ec_boost;
	uint8_t data_buf[8];

	uint8_t byte_ctr = 0;
	uint32_t tx_buf_len = 5;

	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = SLAVE_FRAME;
	tx_buf[2] = tx_buf_len;
	tx_buf[3] = (uint8_t)CmdIdx::MOVE_TO_TIP;

	// CRC
	crc8_atm(tx_buf, tx_buf_len);

	// Write byte to buffer
	while (byte_ctr != tx_buf_len) {
		byte_ctr += port->write_some(boost::asio::buffer(tx_buf, tx_buf_len), ec_boost);

		if (ec_boost) {
			*p_err = PtProtocolErr::TRANSMIT_ERROR;
			return;
		}
	}

	// Read response
	read(rx_buf, data_buf, p_err);
	if (err_get(p_err)) {
		return;
	}
}

void PtMasterProtocol::read(uint8_t _rx_buf[], uint8_t d_reg_buf[], PtProtocolErr* p_err) {
	uint8_t rx_byte;
	bool local_err = false;
	;
	State state = State::SYNC;
	uint32_t msg_length;
	CmdIdx reg;
	uint8_t d_reg;
	uint8_t d_reg_size;
	int8_t d_reg_idx;
	uint8_t crc;

	uint32_t byte_ctr = 0;

	while (1) {
		port->read_some(boost::asio::buffer(&rx_byte, 1), ec);

		// TODO change for specific error
		if (!ec) {
			_rx_buf[byte_ctr] = rx_byte;

			switch (state) {
				case State::SYNC:
					// prevMillis = millis();

					if (rx_byte != SYNC_FRAME) {
						local_err = true;
						break;
					}
					state = State::SLAVE;
					break;

				case State::SLAVE:

					if (rx_byte != MASTER_FRAME) {
						local_err = true;
						break;
					}
					state = State::MSG_LENGTH;
					break;

				case State::MSG_LENGTH:
					msg_length = rx_byte;
					if (msg_length < 5) {
						local_err = true;
						break;
					}
					state = State::REGISTER;
					break;

				case State::REGISTER:

					reg = (CmdIdx)rx_byte;
					if (reg >= CmdIdx::SIZE) {
						local_err = true;
						break;
					}

					if (reg == CmdIdx::CMD_SUCCESSFUL) {
						state = State::CRC;
					} else {
						state = State::DATA_REGISTER;
					}
					break;

				case State::DATA_REGISTER:
					d_reg = rx_byte;
					if (d_reg >= (uint32_t)DataRegIdx::SIZE) {
						local_err = true;
						break;
					}
					d_reg_size = data_reg[d_reg].size;
#if PSNIP_ENDIAN_LITTLE
					d_reg_idx = d_reg_size - 1;  // Little endian
#elif PSNIP_ENDIAN_BIG
					d_reg_idx = 0;  // Big endian
#else
#error Endianness not supported
#endif

					state = State::DATA_REGISTER_STORE;
					break;

				case State::DATA_REGISTER_STORE:
					d_reg_buf[d_reg_idx] = rx_byte;

#if PSNIP_ENDIAN_LITTLE
					d_reg_idx--;  // Big endian
					if (d_reg_idx < 0) {
#elif PSNIP_ENDIAN_BIG
					d_reg_idx++;  // Big endian
					if (d_reg_idx >= d_reg_size) {
#else
#error Endianness not supported
#endif
						state = State::CRC;
					}
					break;

				case State::CRC:
					// TODO check for crc
					crc = rx_byte;

					// calculate crc for comparison
					crc8_atm(_rx_buf, msg_length);

					if (crc != _rx_buf[msg_length - 1]) {
						std::cout << "BAD CRC" << std::endl;
						*p_err = PtProtocolErr::BAD_CRC;
						return;
					}

					// Start command
					switch ((CmdIdx)reg) {
						case CmdIdx::ERROR_PT:
							std::cout << "read PIPETTE_TOOL_ERROR" << std::endl;
							*p_err = PtProtocolErr::PIPETTE_TOOL_ERROR;
							return;

						case CmdIdx::ERROR_PT_PROTOCOL:
							std::cout << "read PROTOCOL_ERROR" << std::endl;
							*p_err = PtProtocolErr::PROTOCOL_ERROR;
							return;

						default:
							return;
					}

					break;
			}
			byte_ctr++;

			if (local_err) {
				std::cout << "read RECEIVE_ERROR" << std::endl;
				*p_err = PtProtocolErr::RECEIVE_ERROR;
				break;
			}
		}
	}
}

void PtMasterProtocol::store_data_to_buf(uint8_t _tx_buf[], uint8_t data_buf[], uint8_t buf_len) {
#if PSNIP_ENDIAN_LITTLE

	uint32_t data_buf_ctr = buf_len - 1;

	for (int32_t i = 0; i < buf_len; i++) {
		_tx_buf[i] = data_buf[data_buf_ctr];
		data_buf_ctr--;
	}
#elif PSNIP_ENDIAN_BIG
	for (int32_t i = 0; i < buf_len; i++) {
		_tx_buf[i] = data_buf[i];
	}
#else
#error Endianness not supported
#endif
}

#endif  // __linux__
