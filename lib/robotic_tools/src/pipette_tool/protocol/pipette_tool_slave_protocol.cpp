/*
 * drv8880_slave_protocol.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: robotic_tools
 */


#if defined(ARDUINO)

#include <robotic_tools/pipette_tool/protocol/pipette_tool_slave_protocol.h>
#include <robotic_tools/error_handle/error_handle.h>


PipetteToolSlaveProtocol::PipetteToolSlaveProtocol(PipetteToolSlaveRegister* tool_reg,
												   StepperMotorSlaveRegister* motor_reg,
												   StepperDriverRegister* driver_reg,
												   TrajectorySlaveRegister* trajectory_reg,
												   LeadscrewSlaveRegister* leadscrew_reg,
												   uint8_t slave_addr)
:PipetteToolBaseProtocol(slave_addr), tool_reg(tool_reg), motor_reg(motor_reg), driver_reg(driver_reg), trajectory_reg(trajectory_reg), leadscrew_reg(leadscrew_reg)
{}


uint8_t* PipetteToolSlaveProtocol::read_command(uint8_t data) {


	bool valid_datagram;


	valid_datagram = read_datagram(data);

	if (!valid_datagram) {
		return nullptr;
	}

	uint8_t offset_reg = (datagram[REGISTER_IDX]  & REGISTER_BM) >> REGISTER_BP;
	RegisterRW rw = RegisterRW((datagram[REGISTER_IDX]  & REGISTER_RW_BM) >> REGISTER_RW_BP);

	uint32_t data_frame = unpackDataFrame();


	RegisterType reg_type = get_register_type(offset_reg);
	uint8_t reg = get_register(offset_reg);


	switch (reg_type) {

		case RegisterType::PIPETTE_TOOL:
			if (rw == RegisterRW::REGISTER_WRITE) {
				tool_reg->set_register(PipetteToolSlaveRegister::Registers(reg), data_frame);
				if(p_err->get()) {goto error;}
			}
			data_frame = tool_reg->get_register(PipetteToolSlaveRegister::Registers(reg));
			break;

		case RegisterType::STEPPER_MOTOR:

			if (rw == RegisterRW::REGISTER_WRITE) {
				motor_reg->set_register(StepperMotorSlaveRegister::Registers(reg), data_frame);
				if(p_err->get()) {goto error;}
			}
			data_frame = motor_reg->get_register(StepperMotorSlaveRegister::Registers(reg));
			break;

		case RegisterType::STEPPER_DRIVER:

			if (rw == RegisterRW::REGISTER_WRITE) {
				driver_reg->set_register(reg, data_frame);
				if(p_err->get()) {goto error;}
			}
			data_frame = driver_reg->get_register(reg);
			break;

		case RegisterType::TRAJECTORY:
			if (rw == RegisterRW::REGISTER_WRITE) {
				trajectory_reg->set_register(TrajectorySlaveRegister::Registers(reg), data_frame);
				if(p_err->get()) {goto error;}
			}
			/*
			if(TrajectorySlaveRegister::Registers(reg) == TrajectorySlaveRegister::Registers::BUF_ADD)
			{
				return nullptr;	// Hack for improving speed
			}
			*/

			data_frame = trajectory_reg->get_register(TrajectorySlaveRegister::Registers(reg));
			break;

		case RegisterType::LEADSCREW:
			if (rw == RegisterRW::REGISTER_WRITE) {
				leadscrew_reg->set_register(LeadscrewSlaveRegister::Registers(reg), data_frame);
				if(p_err->get()) {goto error;}
			}
			data_frame = leadscrew_reg->get_register(LeadscrewSlaveRegister::Registers(reg));
			break;

		default:
			/* TODO throw error */
			goto error;
	}

	if(p_err->get()) {
		goto error;
	}

	setDataFrame(data_frame);

	crc8_atm(datagram, DATAGRAM_SIZE);

	return datagram;


error:
	uint32_t err_code;

	/* Change register to error register	*/
	datagram[REGISTER_IDX] = OFFSET_REG_ERROR;

	err_code = p_err->getErrCode();

	/* Populate Data */
	setDataFrame(err_code);

	crc8_atm(datagram, DATAGRAM_SIZE);
	return datagram;
}


#endif



