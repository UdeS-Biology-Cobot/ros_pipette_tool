/*
 * pipette_tool.h
 *
 *  Created on: May 24, 2018
 *      Author: robotic_tools
 */

#ifndef PIPETTE_TOOL_H_
#define PIPETTE_TOOL_H_


#include <robotic_tools/stepper_motor/stepper_motor.h>
#include <robotic_tools/leadscrew/leadscrew.h>
#include <robotic_tools/trajectory/trajectory.h>
#include <robotic_tools/stepper_driver/stepper_driver.h>

/*
#include "stepper_motor.h"
#include "leadscrew.h"
#include "trajectory.h"
#include "stepper_driver.h"
*/

class PipetteTool
{




public:

	enum class Direction : bool
	{
		ASPIRATE,
		DISPENSE
	};

	enum class Type : uint8_t
	{
		GILSON_P200,
	};



	//uint32_t calculate_steps;
	void move(Direction dir, float uL);

	PipetteTool(StepperMotor* motor, StepperDriver* driver, Leadscrew* leadscrew, Trajectory* trajectory, Type type, uint32_t serial);

private:
	float get_distance(float uL);	/* return in mm*/


protected:



public:


	StepperMotor* motor;
	StepperDriver* driver;
	Leadscrew* leadscrew;
	Trajectory* trajectory;
	Type type;
	uint32_t serial;
};










#endif /* PIPETTE_TOOL_H_ */
