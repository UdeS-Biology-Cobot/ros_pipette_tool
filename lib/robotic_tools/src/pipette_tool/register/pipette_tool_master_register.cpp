/*
 * stepper_motor_slave_register.cpp
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */




#include <robotic_tools/pipette_tool/register/pipette_tool_master_register.h>

PipetteToolMasterRegister::PipetteToolMasterRegister()
{}


uint32_t PipetteToolMasterRegister::get_register(Registers reg)
{
	uint32_t data;

	switch (reg) {
		case Registers::TYPE:
			data = datareg_type;
			break;

		case Registers::SERIAL_NUMBER:
			data = datareg_serial;
			break;

		case Registers::DRIVER:
			data = (uint32_t)datareg_driver_type;
			break;

		default:
			/* TODO add erroe */
			return 0;
	}

	return data;
}


void PipetteToolMasterRegister::set_register(Registers reg, uint32_t data)
{
	switch (reg) {
		case Registers::TYPE:
			datareg_type = data;
			break;

		case Registers::SERIAL_NUMBER:
			datareg_serial = data;
			break;

		case Registers::DRIVER:
			datareg_driver_type = StepperDriver::Type(data);
			break;


		default:
			/* TODO add erroe */
			return;
	}
}
