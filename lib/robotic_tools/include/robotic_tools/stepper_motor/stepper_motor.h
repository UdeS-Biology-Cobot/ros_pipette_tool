/*
 * stepper_motor.h
 *
 *  Created on: Jun 17, 2018
 *      Author: robotic_tools
 */

#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_


#include <stdint.h>


class StepperMotor
{
public:
	StepperMotor(uint16_t steps, uint32_t max_vel, uint32_t max_acc);


	uint16_t steps;
	uint32_t max_vel;
	uint32_t max_acc;

};


#endif /* STEPPER_MOTOR_H_ */
