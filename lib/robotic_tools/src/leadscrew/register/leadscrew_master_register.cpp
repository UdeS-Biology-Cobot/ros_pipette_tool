/*
 * stepper_motor_slave_register.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */




#include <robotic_tools/leadscrew/register/leadscrew_master_register.h>

LeadscrewMasterRegister::LeadscrewMasterRegister()
{}


uint32_t LeadscrewMasterRegister::get_register(Registers reg)
{
	uint32_t data;

	switch (reg) {
		case Registers::THREAD_DIR:
			data = *(uint32_t*)&datareg_thread_dir;
			break;

		case Registers::LEAD:
			data = *(uint32_t*)&datareg_lead;
			break;



		default:
			/* TODO add erroe */
			return 0;
	}

	return data;
}


void LeadscrewMasterRegister::set_register(Registers reg, uint32_t data)
{
	switch (reg) {
		case Registers::THREAD_DIR:
			datareg_thread_dir = *(Leadscrew::ThreadDir*)&data;
			break;

		case Registers::LEAD:
			datareg_lead = *(float*)&data;
			break;



		default:
			/* TODO add erroe */
			return;
	}
}


float LeadscrewMasterRegister::get_lead()
{
	return datareg_lead;
}
/*
void LeadscrewMasterRegister::set_lead(float data)
{
	datareg_lead = data;
}
*/
Leadscrew::ThreadDir LeadscrewMasterRegister::get_thread_dir()
{
	return datareg_thread_dir;
}
/*
void LeadscrewMasterRegister::set_thread_dir(Leadscrew::ThreadDir data)
{
	datareg_thread_dir = data;
}
*/


