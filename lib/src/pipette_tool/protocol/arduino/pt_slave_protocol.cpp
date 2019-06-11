/*
 * pt_protocol.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: biobot
 */

#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)

#include <robotic_tool/pipette_tool/protocol/arduino/pt_slave_protocol.h>



PtSlaveProtocol::PtSlaveProtocol(UartSerial* port,
		     uint32_t  baudrate,
			 PipetteTool* pipette_tool,
			 PtProtocolErr* p_err)
: port(port),
  baudrate(baudrate),
  pipette_tool(pipette_tool)
{
	PipetteToolErr err_pt = PipetteToolErr::NONE;


	port->begin(baudrate);

	pipette_tool->init(&err_pt);
	if (pipette_tool->err_get(&err_pt)) {
		*p_err = PtProtocolErr::PIPETTE_TOOL_ERROR;
		return;
	}

}





void PtSlaveProtocol::loop()
{

	//delayMicroseconds(5*buf_len + 29);
	PipetteToolErr err_pt = PipetteToolErr::NONE;

	uint32_t prevMillis;
	uint32_t currMillis;
	uint32_t diffMillis;

	UartSerialErr err_uart = UartSerialErr::NONE;

	uint32_t byte_idx = 0;
	uint8_t rx_byte;


	// Messages
	uint8_t msg_length;
	CmdIdx reg;
	uint8_t d_reg;
	uint8_t st[8][8];



	uint8_t st_idx;
	uint8_t d_reg_size;
	int8_t d_reg_idx;


	bool local_err = false;
	State state = State::SYNC;


	/*
	currMillis = millis();
	diffMillis = currMillis - prevMillis;

	if (diffMillis >= READ_TIMOUT_MS) {
	*/
	uint32_t nl;
	int32_t signed_nl;
	double speed;
	PipetteTool::DisplacementDirection pt_dir;
	Direction dir;
	uint32_t seq_number;
	uint8_t* data;
	uint32_t sn;
	uint8_t crc;



	while (1) {
		state = State::SYNC;
		byte_idx = 0;
		st_idx = 0;
		//Serial.println("---------------------------------------------------------");


		while(1) {

			// Timeout if message is incomplete
			if (state >= State::SLAVE) {
				currMillis = millis();
				diffMillis = currMillis - prevMillis;
				if (diffMillis >= READ_TIMOUT_MS) {
					Serial.println("TIMEOUT");
					break;
				}
			}

			if (port->available() > 0) {
				local_err = false;
				rx_byte = port->read(&err_uart);
				if (port->err_get(&err_uart)) {
					Serial.println("UART ERROR !!!!!!!!");
					break;
				}

				rx_buf[byte_idx] = rx_byte;

				switch (state) {
					case State::SYNC:
						prevMillis = millis();
						//Serial.println("SYNC");
						if (rx_byte != SYNC_FRAME) {
							Serial.println("---ERROR--- SYNC_FRAME");
							local_err = true; break;
						}
						state = State::SLAVE;
						break;

					case State::SLAVE:
						//Serial.println("SLAVE");
						if (rx_byte != SLAVE_FRAME) {
							Serial.println("---ERROR--- SLAVE_FRAME");
							local_err = true;
							break;
						}
						state = State::MSG_LENGTH;
						break;

					case State::MSG_LENGTH:
						//Serial.println("MSG_LENGTH");
						msg_length = rx_byte;
						if (msg_length < 5) {
							Serial.println("---ERROR--- MSG_LENGTH");
							local_err = true;
							break;
						}
						//Serial.print("MSG_LENGTH = ");
						//Serial.println(msg_length);
						state = State::REGISTER;
						break;

					case State::REGISTER:
						//Serial.println("REGISTER");
						reg = (CmdIdx)rx_byte;
						if (reg >= CmdIdx::SIZE) {
							Serial.println("---ERROR--- REGISTER");
							local_err = true;
							break;
						}
						//Serial.print("REGISTER = ");
						//Serial.println((uint32_t)reg);


						// TODO automatise this by adding a register struct with argument param
						if (reg == CmdIdx::HOMING ||
							reg == CmdIdx::EJECT_TIP ||
							reg == CmdIdx::MAX_NL ||
							reg == CmdIdx::MAX_SPEED ||
							reg == CmdIdx::OFFSET_NL ||
							reg == CmdIdx::SERIAL_NUMBER ||
							reg == CmdIdx::REM_ASPIRATE_NL ||
							reg == CmdIdx::REM_DISPENSE_NL ||
							reg == CmdIdx::MOVE_TO_TOP ||
							reg == CmdIdx::MOVE_TO_TIP) {
							state = State::CRC;
						} else {
							state = State::DATA_REGISTER;
						}
						break;

					case State::DATA_REGISTER:
						//Serial.println("DATA_REGISTER");
						d_reg = rx_byte;
						if (d_reg >= (uint32_t)DataRegIdx::SIZE) {
							Serial.println("---ERROR--- DATA_REGISTER");
							local_err = true;
							break;
						}
						d_reg_size = data_reg[d_reg].size;
						d_reg_idx = d_reg_size - 1;	// Little endian
						state = State::DATA_REGISTER_STORE;
						break;

					case State::DATA_REGISTER_STORE:
						//Serial.println("DATA_REGISTER_STORE");
						st[st_idx][d_reg_idx] = rx_byte;

						//Serial.print("byte = ");
						//Serial.println(rx_byte, HEX);

						d_reg_idx--;

						if (d_reg_idx < 0) {
							//Serial.print("byte_idx = ");
							//Serial.println(byte_idx);

							if ((int32_t)byte_idx + 1 == (int32_t)msg_length - 1) {
								state = State::CRC;
							} else {
								st_idx++;
								state = State::DATA_REGISTER;
							}
						}
						break;

					case State::CRC:
						//Serial.println("CRC");
						// TODO
						crc = rx_byte;

						// calculate crc for comparison
						crc8_atm(rx_buf, msg_length);

						if (crc != rx_buf[msg_length - 1]) {
							local_err = true;
							break;
						}

						// Start command
						switch ((CmdIdx)reg) {
							case CmdIdx::ASPIRATE:
								// Convert array of bytes to good type
								memcpy(&nl, st[0], sizeof(nl));
								memcpy(&speed, st[1], sizeof(speed));

/*
								Serial.print("rx[0] = ");
								Serial.println(rx_buf[0], HEX);
								Serial.print("rx[1] = ");
								Serial.println(rx_buf[1], HEX);
								Serial.print("rx[2] = ");
								Serial.println(rx_buf[2], HEX);
								Serial.print("rx[3] = ");
								Serial.println(rx_buf[3], HEX);
								Serial.print("rx[4] = ");
								Serial.println(rx_buf[4], HEX);
								Serial.print("rx[5] = ");
								Serial.println(rx_buf[5], HEX);
								Serial.print("rx[6] = ");
								Serial.println(rx_buf[6], HEX);
								Serial.print("rx[7] = ");
								Serial.println(rx_buf[7], HEX);
								Serial.print("rx[8] = ");
								Serial.println(rx_buf[8], HEX);
								Serial.print("rx[9] = ");
								Serial.println(rx_buf[9], HEX);
								Serial.print("rx[10] = ");
								Serial.println(rx_buf[10], HEX);


								Serial.print("nl = ");
								Serial.println(nl);
								Serial.print("speed = ");
								Serial.println(speed);

*/



								err_pt = PipetteToolErr::NONE;
								pipette_tool->aspirate(nl, speed, &err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->aspirate = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {
									send_cmd_successful();
								}
								break;

							case CmdIdx::DISPENSE:
								// Convert array of bytes to good type
								memcpy(&nl, st[0], sizeof(nl));
								memcpy(&speed, st[1], sizeof(speed));

								err_pt = PipetteToolErr::NONE;
								pipette_tool->dispense(nl, speed, &err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->dispense = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {
									send_cmd_successful();
								}
								break;

							case CmdIdx::MULTI_SEQUENCE:




								// Convert array of bytes to good type
								memcpy(&nl, st[0], sizeof(nl));
								memcpy(&speed, st[1], sizeof(speed));
								memcpy(&dir, st[2], sizeof(dir));
								memcpy(&seq_number, st[3], sizeof(seq_number));

								err_pt = PipetteToolErr::NONE;

								pt_dir = dir_to_ptdir(dir);

								pipette_tool->multiple_sequence(nl, speed, pt_dir, seq_number, &err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->multiple_sequence = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {
									send_cmd_successful();
								}
								break;

							case CmdIdx::HOMING:
								err_pt = PipetteToolErr::NONE;
								pipette_tool->do_homing(&err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->do_homing = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {
									send_cmd_successful();
								}
								break;

							case CmdIdx::EJECT_TIP:
								err_pt = PipetteToolErr::NONE;
								pipette_tool->eject_tip(&err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->eject_tip = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {
									send_cmd_successful();
								}
								break;

							case CmdIdx::MAX_NL:
								err_pt = PipetteToolErr::NONE;
								nl = pipette_tool->get_max_nl(&err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->get_max_nl = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {

									data = (uint8_t *) &nl;    // legal and portable C

									send_datareg(reg, DataRegIdx::NANO_LITERS, data);
								}
								break;


							case CmdIdx::REM_ASPIRATE_NL:
								err_pt = PipetteToolErr::NONE;
								nl = pipette_tool->get_rem_aspirate_vol_nl(&err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->get_rem_aspirate_vol_nl = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {

									data = (uint8_t *) &nl;    // legal and portable C

									send_datareg(reg, DataRegIdx::NANO_LITERS, data);
								}
								break;

							case CmdIdx::REM_DISPENSE_NL:
								err_pt = PipetteToolErr::NONE;
								nl = pipette_tool->get_rem_dispense_vol_nl(&err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->get_rem_dispense_vol_nl = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {

									data = (uint8_t *) &nl;    // legal and portable C

									send_datareg(reg, DataRegIdx::NANO_LITERS, data);
								}
								break;


							case CmdIdx::MAX_SPEED:
								err_pt = PipetteToolErr::NONE;
								speed = pipette_tool->get_max_speed();

								data = (uint8_t *) &speed;    // legal and portable C

								send_datareg(reg, DataRegIdx::SPEED, data);
								break;

							case CmdIdx::OFFSET_NL:
								err_pt = PipetteToolErr::NONE;
								signed_nl = pipette_tool->get_offset_nl();

								data = (uint8_t *) &signed_nl;    // legal and portable C
								send_datareg(reg, DataRegIdx::SIGNED_NANO_LITERS, data);

								break;


							case CmdIdx::SERIAL_NUMBER:
								err_pt = PipetteToolErr::NONE;
								sn = pipette_tool->get_serial_number(&err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->get_serial_number = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {
									data = (uint8_t *) &sn;    // legal and portable C
									send_datareg(reg, DataRegIdx::SERIAL_NUMBER, data);
								}
								break;


							case CmdIdx::MOVE_TO_TOP:
								err_pt = PipetteToolErr::NONE;
								pipette_tool->move_to_top(&err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->move_to_top = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {
									send_cmd_successful();
								}
								break;

							case CmdIdx::MOVE_TO_TIP:
								err_pt = PipetteToolErr::NONE;
								pipette_tool->move_to_tip(&err_pt);
								if (pipette_tool->err_get(&err_pt)) {
									Serial.print("---ERROR--- pipette_tool->move_to_tip = ");
									Serial.println((uint32_t)err_pt);
									data = (uint8_t *) &err_pt;    // legal and portable C
									send_datareg(CmdIdx::ERROR_PT, DataRegIdx::PT_ERROR, data);
								} else {
									send_cmd_successful();
								}
								break;
						}
						// Reset state
						local_err = true; // Reset loop
						break;


					default:
						local_err = true;
						break;

				}

				byte_idx++;

				if (local_err) {break;}

				if (state >= State::MSG_LENGTH) {
					if (byte_idx >= msg_length) {break;}
				}
			}
		}
	}
}









void PtSlaveProtocol::send_cmd_successful() {

	uint8_t byte_ctr = 0;
	uint32_t buf_len = 5;


	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = MASTER_FRAME;
	tx_buf[2] = buf_len;
	tx_buf[3] = (uint32_t)CmdIdx::CMD_SUCCESSFUL;

	crc8_atm(tx_buf, buf_len);


	// Write byte to buffer
	while (byte_ctr != buf_len) {
		byte_ctr += port->write(tx_buf, buf_len);
	}
}

/*
void PtSlaveProtocol::send_error_code(PtProtocolErr ec) {

	uint8_t byte_ctr = 0;
	uint32_t buf_len = 5 + sizeof(ec);


	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = MASTER_FRAME;
	tx_buf[2] = buf_len;
	tx_buf[3] = (uint32_t)CmdIdx::ERROR_PT_PROTOCOL;

	// https://stackoverflow.com/a/51473972
	uint8_t* ec_buf = (uint8_t *) &ec;    // legal and portable C
	store_data_to_buf(&tx_buf[4], ec_buf, sizeof(ec));

	crc8_atm(tx_buf, buf_len);


	// Write byte to buffer
	while (byte_ctr != buf_len) {
		byte_ctr += port->write(tx_buf, buf_len);
	}
}

void PtSlaveProtocol::send_error_code(PipetteToolErr ec)
{

	uint8_t byte_ctr = 0;
	uint32_t buf_len = 5 + sizeof(ec);


	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = MASTER_FRAME;
	tx_buf[2] = buf_len;
	tx_buf[3] = (uint32_t)CmdIdx::ERROR_PT;

	// https://stackoverflow.com/a/51473972
	uint8_t* ec_buf = (uint8_t *) &ec;    // legal and portable C
	store_data_to_buf(&tx_buf[4], ec_buf, sizeof(ec));

	crc8_atm(tx_buf, buf_len);


	// Write byte to buffer
	while (byte_ctr != buf_len) {
		byte_ctr += port->write(tx_buf, buf_len);
	}

}
*/

void PtSlaveProtocol::send_datareg(CmdIdx reg_idx, DataRegIdx datareg_idx, uint8_t data_buf[]) {


	uint8_t byte_ctr = 0;
	uint32_t buf_len = 5 + 1 + data_reg[(uint32_t)datareg_idx].size;


	tx_buf[0] = SYNC_FRAME;
	tx_buf[1] = MASTER_FRAME;
	tx_buf[2] = buf_len;
	tx_buf[3] = (uint32_t)reg_idx;

	// Store datareg
	tx_buf[4] = (uint32_t)datareg_idx;
	store_data_to_buf(&tx_buf[5], data_buf, data_reg[(uint32_t)datareg_idx].size);

	crc8_atm(tx_buf, buf_len);

/*
	Serial.print("tx[0] = ");
	Serial.println(tx_buf[0], HEX);
	Serial.print("tx[1] = ");
	Serial.println(tx_buf[1], HEX);
	Serial.print("tx[2] = ");
	Serial.println(tx_buf[2], HEX);
	Serial.print("tx[3] = ");
	Serial.println(tx_buf[3], HEX);
	Serial.print("tx[4] = ");
	Serial.println(tx_buf[4], HEX);
	Serial.print("tx[5] = ");
	Serial.println(tx_buf[5], HEX);
	Serial.print("tx[6] = ");
	Serial.println(tx_buf[6], HEX);
	Serial.print("tx[7] = ");
	Serial.println(tx_buf[7], HEX);
	Serial.print("tx[8] = ");
	Serial.println(tx_buf[8], HEX);
	Serial.print("tx[9] = ");
	Serial.println(tx_buf[9], HEX);
	Serial.print("tx[10] = ");
	Serial.println(tx_buf[10], HEX);
*/

	// Write byte to buffer
	while (byte_ctr != buf_len) {
		byte_ctr += port->write(tx_buf, buf_len);
	}
}


// Little endian to big endian
void PtSlaveProtocol::store_data_to_buf(uint8_t _tx_buf[], uint8_t data_buf[], uint8_t buf_len)
{
	uint32_t data_buf_ctr = buf_len - 1;

	for(int32_t i = 0; i < buf_len; i++) {
		_tx_buf[i] = data_buf[data_buf_ctr];
		data_buf_ctr--;
	}
}


PipetteTool::DisplacementDirection PtSlaveProtocol::dir_to_ptdir(Direction dir)
{
	if (dir == Direction::ASPIRATE) {
		return PipetteTool::DisplacementDirection::ASPIRATE;
	} else {
		return PipetteTool::DisplacementDirection::DISPENSE;
	}
}


#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO

