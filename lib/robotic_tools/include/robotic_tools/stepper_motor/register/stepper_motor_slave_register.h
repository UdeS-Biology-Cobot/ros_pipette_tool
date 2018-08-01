/*
 * stepper_motor_slave_register.h
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */

#ifndef STEPPER_MOTOR_SLAVE_REGISTER_H_
#define STEPPER_MOTOR_SLAVE_REGISTER_H_

#include <robotic_tools/stepper_motor/stepper_motor.h>
#include <robotic_tools/stepper_motor/register/stepper_motor_base_register.h>


class StepperMotorSlaveRegister : public StepperMotorBaseRegister
{
public:

	StepperMotorSlaveRegister(StepperMotor* motor);

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;





private:
	StepperMotor* motor;




};

#endif /* STEPPER_MOTOR_SLAVE_REGISTER_H_ */
