/*
 * stepper_driver_register.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: robotic_tools
 */

#include <robotic_tools/stepper_driver/register/stepper_driver_register.h>
//#include "stepper_driver_register.h"

StepperDriverRegister::StepperDriverRegister(DRV8880SlaveRegister* driver_reg)
:driver(driver_reg->driver), driver_reg(driver_reg), DRIVER_REGISTER_SIZE((uint8_t)driver_reg->Registers::SIZE)
{
}

#if defined(ARDUINO)

void StepperDriverRegister::set_register(uint8_t reg, uint32_t data)
{

	switch(driver.type) {

		case driver.Type::DRV8880:
			((DRV8880SlaveRegister*)driver_reg)->set_register(DRV8880SlaveRegister::Registers(reg), data);

	};



}



uint32_t StepperDriverRegister::get_register(uint8_t reg)
{
	switch(driver.type) {

		case driver.Type::DRV8880:
			return ((DRV8880SlaveRegister*)driver_reg)->get_register(DRV8880SlaveRegister::Registers(reg));

	};

	/*TODO set error*/
	return 0;
}

#endif
