/*
 * protocol_drv8880.cpp
 *
 *  Created on: May 28, 2018
 *      Author: robotic_tools
 */


#include <robotic_tools/error_handle/error_handle.h>
#include <robotic_tools/stepper_driver/register/drv8880_slave_register.h>


#if defined(ARDUINO)


DRV8880SlaveRegister::DRV8880SlaveRegister(DRV8880* driver)
:driver(driver)
{}


DRV8880SlaveRegister::~DRV8880SlaveRegister()
{}










void DRV8880SlaveRegister::set_register(Registers reg, uint32_t data)
{
	switch (reg) {
		case Registers::CONFIG:
			set_register_config(data);
			break;
		case Registers::MICROSTEP:
			/* TODO set error can only be set in CONFIG register from microstep_mode */
			break;

		default:
			/* TODO add erroe */
			return;
	}
}

uint32_t DRV8880SlaveRegister::get_register(Registers reg)
{
	switch (reg) {
		case Registers::CONFIG:
			return get_register_config();

		case Registers::MICROSTEP:
			return driver->getMicrostep();

		default:
			/* TODO add erroe */
			return 0;
	}
}



void DRV8880SlaveRegister::set_register_config(uint32_t data)
{
	DRV8880::MicrostepMode mres_mode;
	DRV8880::Current current;
	DRV8880::Direction direction;


	/* Decode data */
	mres_mode  = decode_microstep_mode(data);
	if(p_err->get()) {return;}
	current  = decode_current(data);
	if(p_err->get()) {return;}
	direction  = decode_direction(data);
	if(p_err->get()) {return;}

	/* Check if already set*/
	if  (mres_mode != driver->getMicrostepMode()) {
		driver->setMicrostepMode(mres_mode);
		if(p_err->get()) {return;}
	}
	if  (current != driver->getCurrent()) {
		driver->setCurrent(current);
		if(p_err->get()) {return;}
	}
	if  (direction != driver->getDirection()) {
		driver->setDirection(direction);
		if(p_err->get()) {return;}
	}
}


uint32_t DRV8880SlaveRegister::get_register_config()
{
	uint32_t data = 0;

	DRV8880::MicrostepMode mres_mode;
	DRV8880::Current current;
	DRV8880::Direction direction;

	mres_mode = driver->getMicrostepMode();
	current = driver->getCurrent();
	direction = driver->getDirection();


	data |= encode_microstep_mode(mres_mode);
	data |= encode_current(current);
	data |= encode_direction(direction);

	return data;
}


#endif


