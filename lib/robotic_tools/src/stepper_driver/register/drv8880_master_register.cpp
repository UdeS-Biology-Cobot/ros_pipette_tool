/*
 * drv8880_master_register.cpp
 *
 *  Created on: Jun 22, 2018
 *      Author: robotic_tools
 */


#include <robotic_tools/stepper_driver/register/drv8880_master_register.h>
#include <robotic_tools/stepper_driver/arduino/drv8880.h>
#include <robotic_tools/error_handle/error_handle.h>


DRV8880MasterRegister::DRV8880MasterRegister()
{}


void DRV8880MasterRegister::set_register(Registers reg, uint32_t datareg)
{
	switch (reg) {
		case Registers::CONFIG:
			data_microstep_mode = decode_microstep_mode(datareg);


			datareg_microstep = DRV8880::getMicrostep(data_microstep_mode);	/* microstep mode set microstep */
			data_current = decode_current(datareg);
			data_direction = decode_direction(datareg);
			break;

		case Registers::MICROSTEP:
			datareg_microstep = datareg;
			break;

		default:
			/* TODO add erroe */
			return;
	}
}



uint32_t DRV8880MasterRegister::get_register(Registers reg)
{
	uint32_t datareg = 0;

	switch (reg) {
		case Registers::CONFIG:
			datareg |= encode_microstep_mode(data_microstep_mode);
			datareg |= encode_current(data_current);
			datareg |= encode_direction(data_direction);
			return datareg;

		case Registers::MICROSTEP:
			datareg |= datareg_microstep;
			return datareg;

		default:
			/* TODO add erroe */
			return 0;
	}
}


/*
void DRV8880MasterRegister::set_mres(DRV8880::Mres data)
{
	data_mres = data;
}


void DRV8880MasterRegister::set_current(DRV8880::Current data)
{
	data_current = data;
}

void DRV8880MasterRegister::set_direction(DRV8880::Direction data)
{
	data_direction = data;
}
*/
uint16_t DRV8880MasterRegister::get_microstep()
{
	return datareg_microstep;
}

DRV8880::MicrostepMode DRV8880MasterRegister::get_microstep_mode()
{
	return data_microstep_mode;
}

DRV8880::Current DRV8880MasterRegister::get_current()
{
	return data_current;
}

DRV8880::Direction DRV8880MasterRegister::get_direction()
{
	return data_direction;
}


