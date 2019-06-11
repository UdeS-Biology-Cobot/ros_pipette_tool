/*
 * drv5023.cpp
 *
 *  Created on: Nov 13, 2018
 *      Author: biobot
 */




#if defined(ARDUINO)

#include <robotic_tool/sensor/drv5023.h>


DRV5023::DRV5023(uint32_t trigger_pin)
: trigger_pin(trigger_pin)
{}

void DRV5023::enable_interrupt(voidFuncPtr callback, uint32_t mode)
{
	if (is_enabled) {
		disable_interrupt();
	}

	attachInterrupt(digitalPinToInterrupt(trigger_pin), callback, mode);
	is_enabled = true;
}


void DRV5023::disable_interrupt()
{
	detachInterrupt(digitalPinToInterrupt(trigger_pin));
	is_enabled = false;
}


bool DRV5023::is_triggered(DRV5023Err* p_err)
{
	int state = digitalRead(trigger_pin);

	if (state == LOW) {
		return true;
	}
	else if (state == HIGH) {
		return false;
	}
	else {
		*p_err = DRV5023Err::DIGITAL_READ_ERROR;
	}

	return false;
}


bool DRV5023::err_get(DRV5023Err* p_err)
{
	if (*p_err == DRV5023Err::NONE) {
		return false;
	}
	return true;
}

void DRV5023::err_clear(DRV5023Err* p_err)
{
	*p_err = DRV5023Err::NONE;
}



#endif	// ARDUINO
