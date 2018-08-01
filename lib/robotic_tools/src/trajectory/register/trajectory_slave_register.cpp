/*
 * stepper_motor_slave_register.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */




#include <robotic_tools/trajectory/register/trajectory_slave_register.h>
#include <robotic_tools/error_handle/error_handle.h>


#if defined(ARDUINO)
TrajectorySlaveRegister::TrajectorySlaveRegister(Trajectory* trajectory)
:trajectory(trajectory)
{}



void TrajectorySlaveRegister::set_register(Registers reg, uint32_t data)
{
	uint8_t param_idx;

	switch (reg) {
		case Registers::COMMAND:
			bool gen;
			gen = decode_command(Command::GENERATE_WITH_PARAMS, data);
			if(p_err->get()) {return;}

			if (gen) {
				trajectory->generate(trajectory->profile_mode, trajectory->get_params());
				if(p_err->get()) {return;}
			}

			gen = decode_command(Command::RUN_TRAJECTORY, data);
			if(p_err->get()) {return;}
			if (gen) {
				trajectory->run();
				if(p_err->get()) {return;}
			}
			break;

		case Registers::PROFILE:

			Trajectory::Profile profile;

			/* Decode data */
			profile  = decode_profile(data);


			if(p_err->get()) {return;}

			/* Check if already set*/
			if  (profile != trajectory->profile_mode) {
				trajectory->profile_mode = profile;
				if(p_err->get()) {return;}
			}
			break;


		case Registers::CPU_FREQUENCY:
			/* TODO add error because only a getter? */
			break;

		case Registers::BUF_LEN:
			/* TODO add error because only a getter? */
			break;

		case Registers::BUF_NEW:

			buf_idx = 0;
			trajectory->set_buf(buf_idx, data);
			if(p_err->get()) {return;}


			break;

		case Registers::BUF_ADD:
			buf_idx++;
			trajectory->set_buf(buf_idx, data);
			if(p_err->get()) {return;}
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

			trajectory->set_params(param_idx, data);
			break;


		default:
			/* TODO add erroe */
			return;
	}
}

uint32_t TrajectorySlaveRegister::get_register(Registers reg)
{
	uint32_t data = 0;
	uint8_t param_idx;

	switch (reg) {
		case Registers::COMMAND:
			data = 0;
			break;


		case Registers::PROFILE:
			Trajectory::Profile profile;

			profile = trajectory->profile_mode;

			data = 0;
			data |= encode_profile(profile);
			break;

		case Registers::CPU_FREQUENCY:
			data = trajectory->f_cpu;
			break;

		case Registers::BUF_LEN:
			data = trajectory->BUF_SIZE;
			break;

		case Registers::BUF_NEW:
			data = trajectory->get_buf()[buf_idx];
			break;

		case Registers::BUF_ADD:
			data = trajectory->get_buf()[buf_idx];
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
			data = trajectory->get_params()[param_idx];
			break;


		default:
			/* TODO add erroe */
			break;
	}

	return data;
}
#endif
