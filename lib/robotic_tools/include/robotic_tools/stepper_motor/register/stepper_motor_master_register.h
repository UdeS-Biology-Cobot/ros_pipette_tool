/*
 * stepper_motor_slave_register.h
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */

#ifndef STEPPER_MOTOR_MASTER_REGISTER_H_
#define STEPPER_MOTOR_MASTER_REGISTER_H_

#include <robotic_tools/stepper_motor/register/stepper_motor_base_register.h>


class StepperMotorMasterRegister : public StepperMotorBaseRegister
{
public:

	StepperMotorMasterRegister();

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;


	uint16_t get_steps();
	uint32_t get_max_vel();
	uint32_t get_max_acc();


private:
	uint16_t datareg_steps = 0;
	uint32_t datareg_max_vel = 0;
	uint32_t datareg_max_acc = 0;
};

#endif /* STEPPER_MOTOR_MASTER_REGISTER_H_ */
