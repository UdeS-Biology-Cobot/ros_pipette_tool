/*
 * stepper_motor_slave_register.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: biobot
 */




#include <robotic_tool/stepper_motor/register/stepper_motor_master_register.h>

StepperMotorMasterRegister::StepperMotorMasterRegister()
{}


uint32_t StepperMotorMasterRegister::get_register(Registers reg)
{
	uint32_t data;

	switch (reg) {
		case Registers::STEPS:
			data = *(uint32_t*)&datareg_steps;
			break;

		case Registers::MAX_VEL:
			data = *(uint32_t*)&datareg_max_vel;
			break;

		case Registers::MAX_ACC:
			data = *(uint32_t*)&datareg_max_acc;
			break;

		default:
			/* TODO add erroe */
			return 0;
	}

	return data;
}


void StepperMotorMasterRegister::set_register(Registers reg, uint32_t data)
{
	switch (reg) {
		case Registers::STEPS:
			datareg_steps = *(uint16_t*)&data;
			break;

		case Registers::MAX_VEL:
			datareg_max_vel = *(uint32_t*)&data;
			break;

		case Registers::MAX_ACC:
			datareg_max_acc = *(uint32_t*)&data;
			break;

		default:
			/* TODO add erroe */
			return;
	}
}






uint16_t StepperMotorMasterRegister::get_steps()
{
	return datareg_steps;
}
uint32_t StepperMotorMasterRegister::get_max_vel()
{
	return datareg_max_vel;
}

uint32_t StepperMotorMasterRegister::get_max_acc()
{
	return datareg_max_acc;
}





