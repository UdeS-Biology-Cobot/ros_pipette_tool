/*
 * stepper_motor_slave_register.h
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */

#ifndef PIPETTE_TOOL_SLAVE_REGISTER_H_
#define PIPETTE_TOOL_SLAVE_REGISTER_H_

#include <robotic_tools/pipette_tool/register/pipette_tool_base_register.h>
#include <robotic_tools/pipette_tool/pipette_tool.h>

//#include <robotic_tools/stepper_driver/register/stepper_driver_register.h>

class PipetteToolSlaveRegister : public PipetteToolBaseRegister
{
public:

	PipetteToolSlaveRegister(PipetteTool* pipette, StepperDriver::Type driver_type);

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;





private:
	PipetteTool* pipette;

public:
	const StepperDriver::Type driver_type;


};

#endif /* PIPETTE_TOOL_SLAVE_REGISTER_H_ */
