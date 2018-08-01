/*
 * drv8880_slave_protocol.h
 *
 *  Created on: Jun 6, 2018
 *      Author: robotic_tools
 */

#ifndef PIPETTE_TOOL_SLAVE_PROTOCOL_H_
#define PIPETTE_TOOL_SLAVE_PROTOCOL_H_


#include <robotic_tools/leadscrew/register/leadscrew_slave_register.h>
#include <robotic_tools/pipette_tool/protocol/pipette_tool_base_protocol.h>
#include <robotic_tools/stepper_motor/register/stepper_motor_slave_register.h>

#include <robotic_tools/pipette_tool/register/pipette_tool_slave_register.h>
#include <robotic_tools/trajectory/register/trajectory_slave_register.h>

#include <robotic_tools/stepper_driver/register/stepper_driver_register.h>

class PipetteToolSlaveProtocol : public PipetteToolBaseProtocol
{


public:

#ifdef MACRO
	struct PipetteToolConfig {
		/* MOTOR CONFIG */
		uint16_t motor_step;
		uint32_t motor_vel_max;
		uint32_t motor_acc_max;


		/* LEADSCREW CONFIG */
		Leadscrew::ThreadDir leadscrew_thread_dir;
		uint64_t leadscrew_travel_per_turn_fm;	/* Unit in femto meters for best accuracy */

		/* TRAJECTORY */
		WaveformGeneration::Mode waveform_mode;

		/* */

	};
#endif


	PipetteToolSlaveProtocol(PipetteToolSlaveRegister* tool_reg,
							 StepperMotorSlaveRegister* motor_reg,
							 StepperDriverRegister* driver_reg,
							 TrajectorySlaveRegister* trajectory_reg,
							 LeadscrewSlaveRegister* leadscrew_reg,
							 uint8_t slave_addr);

	uint8_t* read_command(uint8_t data);


	PipetteToolSlaveRegister* tool_reg;
	StepperMotorSlaveRegister* motor_reg;
	StepperDriverRegister* driver_reg;
	TrajectorySlaveRegister* trajectory_reg;
	LeadscrewSlaveRegister* leadscrew_reg;




private:
	//uint32_t buf_idx = 0;

};

#endif /* PIPETTE_TOOL_SLAVE_PROTOCOL_H_ */
