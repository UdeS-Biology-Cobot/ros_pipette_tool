/*
 * stepper_motor_slave_register.h
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */

#ifndef PIPETTE_TOOL_MASTER_REGISTER_H_
#define PIPETTE_TOOL_MASTER_REGISTER_H_

#include <robotic_tools/pipette_tool/register/pipette_tool_base_register.h>


class PipetteToolMasterRegister : public PipetteToolBaseRegister
{
public:

	PipetteToolMasterRegister();

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;




private:
	uint32_t datareg_type = 0;
	uint32_t datareg_serial = 0;

	StepperDriver::Type datareg_driver_type = StepperDriver::Type(0);


};

#endif /* PIPETTE_TOOL_MASTER_REGISTER_H_ */
