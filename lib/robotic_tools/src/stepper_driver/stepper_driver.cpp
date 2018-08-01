/*
 * StepperDriver.cpp
 *
 *  Created on: Jun 18, 2018
 *      Author: robotic_tools
 */


//#include "stepper_driver.h"
#include <robotic_tools/stepper_driver/stepper_driver.h>



StepperDriver::StepperDriver(DRV8880* driver)
:driver(driver), type(Type::DRV8880)
{}

#if defined(ARDUINO)
uint16_t StepperDriver::get_microsteps()
{
	switch(type) {

		case Type::DRV8880:
			return ((DRV8880*)driver)->getMicrostep();

	};
	/* TODO add error*/
	return 0;

}
#endif
