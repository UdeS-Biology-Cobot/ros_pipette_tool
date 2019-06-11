/*
 * stepper_motor_register.h
 *
 *  Created on: Jun 22, 2018
 *      Author: biobot
 */

#ifndef STEPPER_MOTOR_BASE_REGISTER_H_
#define STEPPER_MOTOR_BASE_REGISTER_H_



#include <stdint.h>


class StepperMotorBaseRegister
{
public:

	StepperMotorBaseRegister();


	enum class Registers {
		STEPS,
		MAX_VEL,
		MAX_ACC,

		SIZE	/* ALWAYS AT END  */
	};


	virtual uint32_t get_register(Registers) = 0;
	virtual void set_register(Registers, uint32_t) = 0;


};

#endif /* STEPPER_MOTOR_BASE_REGISTER_H_ */
