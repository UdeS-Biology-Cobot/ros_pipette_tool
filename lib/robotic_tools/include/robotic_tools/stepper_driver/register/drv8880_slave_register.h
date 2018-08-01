/*
 * protocol_drv8880.h
 *
 *  Created on: May 28, 2018
 *      Author: robotic_tools
 */

#ifndef DRV8880_SLAVE_REGISTER_H_
#define DRV8880_SLAVE_REGISTER_H_

#include <robotic_tools/stepper_driver/arduino/drv8880.h>
//#include <robotic_tools/trajectory/register/trajectory_register.h>
#include <robotic_tools/stepper_driver/register/drv8880_base_register.h>




class DRV8880SlaveRegister : public DRV8880BaseRegister
{
public:

	DRV8880SlaveRegister(DRV8880* driver);
	~DRV8880SlaveRegister();

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;




	DRV8880* driver;

private:
	void set_register_config(uint32_t);
	uint32_t get_register_config();


};

#endif /* DRV8880_SLAVE_REGISTER_H_ */
