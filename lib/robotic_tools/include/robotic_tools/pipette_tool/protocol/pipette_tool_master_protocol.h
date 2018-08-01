/*
 * drv8880_master_protocol.h
 *
 *  Created on: Jun 6, 2018
 *      Author: robotic_tools
 */

#ifndef PIPETTE_TOOL_MASTER_PROTOCOL_H_
#define PIPETTE_TOOL_MASTER_PROTOCOL_H_



#include <robotic_tools/stepper_motor/register/stepper_motor_master_register.h>

#include <robotic_tools/leadscrew/register/leadscrew_master_register.h>
#include <robotic_tools/pipette_tool/protocol/pipette_tool_base_protocol.h>
#include <robotic_tools/pipette_tool/register/pipette_tool_master_register.h>

#include <robotic_tools/stepper_driver/register/stepper_driver_register.h>
#include <robotic_tools/trajectory/register/trajectory_master_register.h>

#include <functional>
#include <iostream>




#include <boost/function.hpp>



class PipetteToolMasterProtocol : public PipetteToolBaseProtocol
{





	//const uint8_t OFFSET_REG_STEPPER_MOTOR =;



public:

	//typedef std::function<uint8_t(uint8_t*, uint8_t)> CallbackWrite;
	//typedef std::function<uint8_t(void)> CallbackRead;

	PipetteToolMasterProtocol(uint8_t slave_addr, boost::function<uint8_t(uint8_t*, uint8_t)> const &cb_write , boost::function<uint8_t(void)> const &cb_read);
	~PipetteToolMasterProtocol();

	boost::function<uint8_t(uint8_t*, uint8_t)> cb_write;
	boost::function<uint8_t(void)> cb_read;

	//CallbackWrite cb_write;
	//CallbackRead  cb_read;

	uint8_t* command(uint8_t offset_reg);						/* Get Command 	*/
	uint8_t* command(uint8_t offset_reg, uint32_t data);		/* Set Command	*/
	void setFrameRegister(uint8_t offset_reg, RegisterRW rw);

	void set_direction(PipetteTool::Direction dir);
	bool read_response(uint8_t data);



	void cmd_move(PipetteTool::Direction dir, double uL, Trajectory::Profile profile, uint32_t params[]);

	void cmd_write(uint8_t* buf);
	void cmd_wrtie_only(uint8_t* buf);

	double get_distance(double uL);
	void get_conf();
	void set_driver_default();

protected:
	PipetteToolMasterRegister pipette_reg = PipetteToolMasterRegister();
	StepperMotorMasterRegister motor_reg = StepperMotorMasterRegister();
	LeadscrewMasterRegister leadscrew_reg = LeadscrewMasterRegister();
	TrajectoryMasterRegister trajectory_reg = TrajectoryMasterRegister();


	StepperDriver::Type driver_type;

	void* driver_reg;


	//static const uint16_t BUF_SIZE = 512;
	//uint16_t buf[BUF_SIZE];
	uint16_t* trajectory_buf = nullptr;
	uint16_t TRAJECTORY_BUF_SIZE;


/////////////////////////////////////////////////////////////////////////
#ifdef MACRO
	/* Command for Register DRV8880 */
	uint8_t* cmd_drv8880(DRV8880::Mres, RegisterRW rw);
	uint8_t* cmd_drv8880(DRV8880::Current, RegisterRW rw);
	uint8_t* cmd_drv8880(DRV8880::Direction, RegisterRW rw);
	uint8_t* cmd_drv8880(DRV8880::Mres, DRV8880::Current, DRV8880::Direction, RegisterRW rw);


	/* Command for Register Trajectory */
	uint8_t* cmd_trajectory_command(TrajectoryMasterRegister::Command, RegisterRW rw);
	uint8_t* cmd_trajectory_profile(Trajectory::Profile, RegisterRW rw);
	uint8_t* cmd_trajectory_param0(uint32_t, RegisterRW rw);
	uint8_t* cmd_trajectory_param1(uint32_t, RegisterRW rw);
	uint8_t* cmd_trajectory_param2(uint32_t, RegisterRW rw);
	uint8_t* cmd_trajectory_param3(uint32_t, RegisterRW rw);

	void setFrameRegisterDRV8880(RegisterRW rw);
	void setFrameRegisterTrajectory(TrajectoryRegister::Registers, RegisterRW rw);
#endif








	/* TODO function for getting all registers */

};


#endif /* PIPETTE_TOOL_MASTER_PROTOCOL_H_ */
