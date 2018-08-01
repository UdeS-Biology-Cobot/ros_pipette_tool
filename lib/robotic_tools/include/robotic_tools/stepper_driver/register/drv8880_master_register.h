/*
 * drv8880_master_register.h
 *
 *  Created on: Jun 22, 2018
 *      Author: robotic_tools
 */

#ifndef DRV8880_MASTER_REGISTER_H_
#define DRV8880_MASTER_REGISTER_H_



//#include <robotic_tools/trajectory/register/trajectory_register.h>
#include <robotic_tools/stepper_driver/register/drv8880_base_register.h>
#include <robotic_tools/stepper_driver/arduino/drv8880.h>


class DRV8880MasterRegister : public DRV8880BaseRegister
{
public:

	DRV8880MasterRegister();

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;

/*
	void set_mres(DRV8880::Mres);
	void set_current(DRV8880::Current);
	void set_direction(DRV8880::Direction);
*/

	DRV8880::MicrostepMode get_microstep_mode();
	DRV8880::Current get_current();
	DRV8880::Direction get_direction();
	uint16_t get_microstep();

private:
	//void set_register_config(uint32_t);
	//uint32_t get_register_config();
	//uint32_t datareg_config = 0;
	//uint32_t datareg_config = 0;

	uint16_t datareg_microstep;
	DRV8880::MicrostepMode  data_microstep_mode;
	DRV8880::Current data_current;
	DRV8880::Direction data_direction;
};


#endif /* DRV8880_MASTER_REGISTER_H_ */
