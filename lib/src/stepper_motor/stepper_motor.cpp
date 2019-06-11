/*
 * stepper_motor.cpp
 *
 *  Created on: Jun 17, 2018
 *      Author: biobot
 */



#include <robotic_tool/stepper_motor/stepper_motor.h>



StepperMotor::StepperMotor(uint16_t steps_per_rev, float max_vel_rps, float max_acc)
:steps_per_rev(steps_per_rev), max_vel_rps(max_vel_rps), max_acc(max_acc)
{}


