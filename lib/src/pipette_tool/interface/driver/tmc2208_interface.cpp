/*
 * tmc2208_interface.cpp
 *
 *  Created on: Dec 1, 2018
 *      Author: biobot
 */

#if defined(ARDUINO)

#include <robotic_tool/pipette_tool/interface/driver/tmc2208_interface.h>


TMC2208Interface::TMC2208Interface(UartSerial* port, uint32_t baudrate, uint32_t dir_pin, uint32_t en_pin, DriverInterfaceErr* p_err)
:TMC2208(port, baudrate, dir_pin, en_pin, (TMC2208Err *)p_err)

{
	if(this->TMC2208::err_get((TMC2208Err *)p_err)) {
		*p_err = modify_err_code((TMC2208Err *)p_err);
	}
}




uint32_t TMC2208Interface::get_max_microstep(DriverInterfaceErr* p_err)
{
	TMC2208Err err_tmc2208 = TMC2208Err::NONE;

	uint32_t microstep = TMC2208::get_max_microstep(&err_tmc2208);
	if(TMC2208::err_get(&err_tmc2208)) {
		*p_err = modify_err_code(&err_tmc2208);
		return 0;
	}

	return microstep;
}

uint32_t TMC2208Interface::get_microstep(DriverInterfaceErr* p_err)
{
	TMC2208Err err_tmc2208 = TMC2208Err::NONE;

	uint32_t microstep = TMC2208::get_microstep(&err_tmc2208);
	if(TMC2208::err_get(&err_tmc2208)) {
		*p_err = modify_err_code(&err_tmc2208);
		return 0;
	}

	return microstep;
}

void TMC2208Interface::set_microstep(uint32_t microstep, DriverInterfaceErr* p_err)
{
	TMC2208Err err_tmc2208 = TMC2208Err::NONE;

	TMC2208::set_microstep(microstep, &err_tmc2208);
	if(TMC2208::err_get(&err_tmc2208)) {
		*p_err = modify_err_code(&err_tmc2208);
	}
}

void TMC2208Interface::set_direction(Direction dir, DriverInterfaceErr* p_err)
{
	TMC2208Err err_tmc2208 = TMC2208Err::NONE;

	TMC2208::RotationDir rot_dir;

	if (dir == Direction::CW) {
		rot_dir = TMC2208::RotationDir::CW;
	} else {
		rot_dir = TMC2208::RotationDir::CCW;
	}

	TMC2208::set_direction(rot_dir, &err_tmc2208);
	if(TMC2208::err_get(&err_tmc2208)) {
		*p_err = modify_err_code(&err_tmc2208);
	}
}

void TMC2208Interface::enable_motor(DriverInterfaceErr* p_err)
{
	TMC2208::enable_motor();
}

void TMC2208Interface::disable_motor(DriverInterfaceErr* p_err)
{
	TMC2208::disable_motor();
}






bool TMC2208Interface::err_get(DriverInterfaceErr* p_err)
{
	if (*p_err == DriverInterfaceErr::NONE) {
		return false;
	}
	return true;
}

void TMC2208Interface::err_clear(DriverInterfaceErr* p_err)
{
	*p_err = DriverInterfaceErr::NONE;
}

DriverInterfaceErr TMC2208Interface::modify_err_code(TMC2208Err* p_err)
{
	switch (*p_err) {
		case TMC2208Err::NONE:
			return DriverInterfaceErr::NONE;

		default:
			return DriverInterfaceErr::INTERNAL_ERROR;
	}
}

#endif // ARDUINO
