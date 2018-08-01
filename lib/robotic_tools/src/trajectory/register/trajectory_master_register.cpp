/*
 * stepper_motor_slave_register.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */




#include <robotic_tools/trajectory/register/trajectory_master_register.h>
#include <robotic_tools/error_handle/error_handle.h>


TrajectoryMasterRegister::TrajectoryMasterRegister()
{}

uint32_t TrajectoryMasterRegister::get_buf_len()
{
	return datareg_buf_len;
}

void TrajectoryMasterRegister::set_register(Registers reg, uint32_t data)
{
	uint8_t param_idx;

	switch (reg) {
		case Registers::COMMAND:
			datareg_command = data;
			break;

		case Registers::PROFILE:
			datareg_profile = data;
			break;

		case Registers::CPU_FREQUENCY:
			datareg_cpu_frequency = data;
			break;

		case Registers::BUF_LEN:
			datareg_buf_len = data;
			break;

		case Registers::BUF_NEW:
			datareg_buf_new = data;
			break;

		case Registers::BUF_ADD:
			datareg_buf_add = data;
			break;

		case Registers::PARAM0:
		case Registers::PARAM1:
		case Registers::PARAM2:
		case Registers::PARAM3:
		case Registers::PARAM4:
		case Registers::PARAM5:
		case Registers::PARAM6:
		case Registers::PARAM7:
		case Registers::PARAM8:
		case Registers::PARAM9:
		case Registers::PARAM10:
		case Registers::PARAM11:
		case Registers::PARAM12:
		case Registers::PARAM13:
		case Registers::PARAM14:
		case Registers::PARAM15:
			param_idx = (uint8_t)reg - (uint8_t)Registers::PARAM0;

			datareg_param[param_idx] = data;
			break;


		default:
			/* TODO add erroe */
			return;
	}
}

uint32_t TrajectoryMasterRegister::get_register(Registers reg)
{
	uint32_t data;
	uint8_t param_idx;

	switch (reg) {
		case Registers::COMMAND:
			data = datareg_command;
			break;

		case Registers::PROFILE:
			data = datareg_profile;
			break;

		case Registers::CPU_FREQUENCY:
			data = datareg_cpu_frequency;
			break;

		case Registers::BUF_LEN:
			data = datareg_buf_len;
			break;

		case Registers::BUF_NEW:
			data = datareg_buf_new;
			break;

		case Registers::BUF_ADD:
			data = datareg_buf_add;
			break;

		case Registers::PARAM0:
		case Registers::PARAM1:
		case Registers::PARAM2:
		case Registers::PARAM3:
		case Registers::PARAM4:
		case Registers::PARAM5:
		case Registers::PARAM6:
		case Registers::PARAM7:
		case Registers::PARAM8:
		case Registers::PARAM9:
		case Registers::PARAM10:
		case Registers::PARAM11:
		case Registers::PARAM12:
		case Registers::PARAM13:
		case Registers::PARAM14:
		case Registers::PARAM15:
			param_idx = (uint8_t)reg - (uint8_t)Registers::PARAM0;

			data = datareg_param[param_idx];
			break;


		default:
			/* TODO add erroe */
			break;
	}
	return data;
}
