/*
 * stepper_motor.h
 *
 *  Created on: Jun 17, 2018
 *      Author: biobot
 */

#ifndef STEPPER_MOTOR_H_
#define STEPPER_MOTOR_H_


#include <stdint.h>


class StepperMotor
{
public:
	StepperMotor(uint16_t steps_per_rev, float max_vel, float max_acc);


	uint16_t steps_per_rev;
	float max_vel_rps;		// RPS
	float max_acc;

};


#endif /* STEPPER_MOTOR_H_ */
