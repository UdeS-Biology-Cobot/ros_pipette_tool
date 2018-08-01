/*
 * stepper_motor.cpp
 *
 *  Created on: Jun 17, 2018
 *      Author: robotic_tools
 */



#include <robotic_tools/stepper_motor/stepper_motor.h>



StepperMotor::StepperMotor(uint16_t steps, uint32_t max_vel, uint32_t max_acc)
:steps(steps), max_vel(max_vel), max_acc(max_acc)
{}


