/*
 * stepper_motor_slave_register.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */




#include <robotic_tools/pipette_tool/register/pipette_tool_slave_register.h>



PipetteToolSlaveRegister::PipetteToolSlaveRegister(PipetteTool* pipette, StepperDriver::Type driver_type)
:pipette(pipette), driver_type(driver_type)
{}


uint32_t PipetteToolSlaveRegister::get_register(Registers reg)
{
	uint32_t data;

	switch (reg) {
		case Registers::TYPE:
			data = (uint32_t)pipette->type;
			break;

		case Registers::SERIAL_NUMBER:
			data = pipette->serial;
			break;

		case Registers::DRIVER:
			data = (uint32_t)driver_type;
			break;



		default:
			/* TODO add erroe */
			return 0;
	}

	return data;
}


void PipetteToolSlaveRegister::set_register(Registers reg, uint32_t data)
{

	/* TODO set error for only read register */
}
