/*
 * stepper_driver_register.h
 *
 *  Created on: Jun 18, 2018
 *      Author: robotic_tools
 */

#ifndef STEPPER_DRIVER_REGISTER_H_
#define STEPPER_DRIVER_REGISTER_H_

#include <robotic_tools/stepper_driver/register/drv8880_master_register.h>
#include <robotic_tools/stepper_driver/register/drv8880_slave_register.h>
#include <robotic_tools/stepper_driver/stepper_driver.h>


class StepperDriverRegister
{
public:

	StepperDriverRegister(DRV8880SlaveRegister* driver_reg);





	void set_register(uint8_t reg, uint32_t data);
	uint32_t get_register(uint8_t reg);


	StepperDriver driver;


	enum class Registers : uint8_t
	{
		REGISTER0,
		REGISTER1,
		REGISTER2,
		REGISTER3,

		SIZE	/* ALWAYS LAST */
	};


private:
	void* driver_reg;


public:
	uint8_t DRIVER_REGISTER_SIZE;

};



#endif /* STEPPER_DRIVER_REGISTER_H_ */
