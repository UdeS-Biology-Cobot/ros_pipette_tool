/*
 * stepper_motor_slave_register.h
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */

#ifndef TRAJECTORY_SLAVE_REGISTER_H_
#define TRAJECTORY_SLAVE_REGISTER_H_

#include <robotic_tools/trajectory/register/trajectory_base_register.h>
#include <robotic_tools/trajectory/trajectory.h>

class TrajectorySlaveRegister : public TrajectoryBaseRegister
{
public:

	TrajectorySlaveRegister(Trajectory* trajectory);

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;





private:
	Trajectory* trajectory;


	uint32_t buf_idx = 0;


};

#endif /* TRAJECTORY_SLAVE_REGISTER_H_ */
