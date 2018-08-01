/*
 * StepperDriver.h
 *
 *  Created on: Jun 18, 2018
 *      Author: robotic_tools
 */

#ifndef STEPPER_DRIVER_H_
#define STEPPER_DRIVER_H_

#include <stdint.h>

#include <robotic_tools/stepper_driver/arduino/drv8880.h>



class StepperDriver
{
public:
	StepperDriver(DRV8880* driver);
	uint16_t get_microsteps();





	enum class Type : uint8_t
	{
		DRV8880,
	};



private:
	void* driver;


public:
	const Type type;
};



#endif /* STEPPER_DRIVER_H_ */
