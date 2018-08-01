/*
 * stepper_motor_register.h
 *
 *  Created on: Jun 22, 2018
 *      Author: robotic_tools
 */

#ifndef LEADSCREW_BASE_REGISTER_H_
#define LEADSCREW_BASE_REGISTER_H_



#include <stdint.h>

class LeadscrewBaseRegister
{
public:

	LeadscrewBaseRegister();


	enum class Registers {
		THREAD_DIR,
		LEAD,

		SIZE	/* ALWAYS AT END */
	};


	virtual uint32_t get_register(Registers) = 0;
	virtual void set_register(Registers, uint32_t) = 0;


};

#endif /* LEADSCREW_BASE_REGISTER_H_ */
