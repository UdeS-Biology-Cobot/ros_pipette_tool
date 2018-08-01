/*
 * stepper_motor_slave_register.h
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */

#ifndef LEADSCREW_MASTER_REGISTER_H_
#define LEADSCREW_MASTER_REGISTER_H_

#include <robotic_tools/leadscrew/register/leadscrew_base_register.h>
#include <robotic_tools/leadscrew/leadscrew.h>

class LeadscrewMasterRegister : public LeadscrewBaseRegister
{
public:

	LeadscrewMasterRegister();

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;


	float get_lead();
	//void set_lead(float data);

	Leadscrew::ThreadDir get_thread_dir();
	//void set_thread_dir(Leadscrew::ThreadDir);

private:
	Leadscrew::ThreadDir datareg_thread_dir;
	float datareg_lead;
};

#endif /* BIOBOT_STEPPER_MOTOR_REGISTER_STEPPER_MOTOR_SLAVE_REGISTER_H_ */
