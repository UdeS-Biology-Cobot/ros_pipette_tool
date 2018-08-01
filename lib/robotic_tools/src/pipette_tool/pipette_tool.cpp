/*
 * pipette_tool.cpp
 *
 *  Created on: May 24, 2018
 *      Author: robotic_tools
 */
#if defined(ARDUINO)
#include <robotic_tools/pipette_tool/pipette_tool.h>
//#include "pipette_tool.h"



PipetteTool::PipetteTool(StepperMotor* motor, StepperDriver* driver, Leadscrew* leadscrew, Trajectory* trajectory, Type type, uint32_t serial)
:motor(motor), driver(driver), leadscrew(leadscrew), trajectory(trajectory), type(type), serial(serial)
{

	/* TODO map ASPIRATE and DISPENSE to good direction (DEPENDS on leadscrew and driver)*/



}



void PipetteTool::move(Direction dir, float uL)
{

	double distance_per_step;
	float travel_distance;
	uint16_t microsteps;
	uint32_t steps;

	/* Calculate distance required for desired volume */
	travel_distance = get_distance(uL);


	/* Calculate number of steps needed for generating trajectory */
	microsteps = driver->get_microsteps();

	distance_per_step = ((motor->steps / leadscrew->lead) / microsteps);

	steps = travel_distance / distance_per_step;


	/* Generate trajectory */

	uint32_t f_min = 500;
	uint32_t f_max = 4800 * microsteps;
	uint32_t slope = 100;

	uint32_t params[4] = {steps, f_min, f_max, slope};

	uint16_t buf_len;
	uint16_t* buf;


	buf_len = trajectory->generate(Trajectory::Profile::CONCAVE_SCURVE_VEL, params);
	buf = trajectory->get_buf();


	/* TODO check for max vel and max accel */

}






float PipetteTool::get_distance(float uL)
{
	switch (type) {
		case Type::GILSON_P200:
			return ((uL + 1.918) / 12.499);
			break;
	}
	/*Throw error if passed this */
	return 0.0;
}
#endif
