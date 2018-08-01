/*
 * stepper_motor_slave_register.h
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */

#ifndef LEADSCREW_SLAVE_REGISTER_H_
#define LEADSCREW_SLAVE_REGISTER_H_


#include <robotic_tools/leadscrew/register/leadscrew_base_register.h>
#include <robotic_tools/leadscrew/leadscrew.h>

class LeadscrewSlaveRegister : public LeadscrewBaseRegister
{
public:

	LeadscrewSlaveRegister(Leadscrew* leadscrew);

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;





private:
	Leadscrew* leadscrew;




};

#endif /* LEADSCREW_SLAVE_REGISTER_H_ */
