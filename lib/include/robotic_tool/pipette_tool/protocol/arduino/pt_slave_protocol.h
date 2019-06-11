/*
 * pt_protocol.h
 *
 *  Created on: Dec 5, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_PROTOCOL_ARDUINO_PT_SLAVE_PROTOCOL_H_
#define LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_PROTOCOL_ARDUINO_PT_SLAVE_PROTOCOL_H_



#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)

#include <robotic_tool/pipette_tool/protocol/pt_protocol_base.h>
#include <robotic_tool/pipette_tool/pipette_tool.h>
#include <robotic_tool/uart/arduino/uart_serial.h>












class PtSlaveProtocol: public PtProtocolBase
{
public:
/*
	enum Frame : uint8_t {
		SYNC,
		SLAVE,
		MSG_LENGTH,
		REGISTER,
	};
*/






	PtSlaveProtocol(UartSerial* port, uint32_t baudrate, PipetteTool* pipette_tool, PtProtocolErr* p_err);

	//void test(storage_type st, float* data);




	void send_cmd_successful();
	//void send_error_code(PtProtocolErr ec);
	//void send_error_code(PipetteToolErr ec);
	void send_datareg(CmdIdx reg_idx, DataRegIdx datareg_idx, uint8_t data_buf[]);
	void store_data_to_buf(uint8_t _tx_buf[], uint8_t data_buf[], uint8_t buf_len);
	void loop();

	UartSerial* port;
	uint32_t baudrate;
	PipetteTool* pipette_tool;

	uint8_t tx_buf[64];
	uint8_t rx_buf[64];



	const uint8_t SLAVE_FRAME	= 0x00;


	const uint32_t READ_TIMOUT_MS = 200;


private:
	PipetteTool::DisplacementDirection dir_to_ptdir(Direction dir);


};


#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO

#endif /* LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_PROTOCOL_ARDUINO_PT_SLAVE_PROTOCOL_H_ */
