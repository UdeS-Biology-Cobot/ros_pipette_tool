/*
 * stepper_motor_slave_register.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */




#include <robotic_tools/leadscrew/register/leadscrew_slave_register.h>

LeadscrewSlaveRegister::LeadscrewSlaveRegister(Leadscrew* leadscrew)
:leadscrew(leadscrew)
{}


uint32_t LeadscrewSlaveRegister::get_register(Registers reg)
{
	uint32_t data;

	switch (reg) {
		case Registers::THREAD_DIR:
			data = *(uint32_t*)&leadscrew->thread_dir;
			break;

		case Registers::LEAD:
			data = *(uint32_t*)&(leadscrew->lead);
			break;


		default:
			/* TODO add erroe */
			return 0;
	}

	return data;
}


void LeadscrewSlaveRegister::set_register(Registers reg, uint32_t data)
{

	/* TODO set error for only read register */
}
