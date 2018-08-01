/*
 * stepper_motor_slave_register.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */




#include <robotic_tools/stepper_motor/register/stepper_motor_slave_register.h>




StepperMotorSlaveRegister::StepperMotorSlaveRegister(StepperMotor* motor)
:motor(motor)
{}



uint32_t StepperMotorSlaveRegister::get_register(Registers reg)
{
	uint32_t data;

	switch (reg) {
		case Registers::STEPS:
			data = motor->steps;
			break;

		case Registers::MAX_VEL:
			data = motor->max_vel;
			break;

		case Registers::MAX_ACC:
			data = motor->max_acc;
			break;

		default:
			/* TODO add erroe */
			return 0;
	}

	return data;
}


void StepperMotorSlaveRegister::set_register(Registers reg, uint32_t data)
{

	/* TODO set error for only read register */
}
