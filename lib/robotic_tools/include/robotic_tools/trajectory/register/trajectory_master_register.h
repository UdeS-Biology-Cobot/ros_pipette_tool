/*
 * stepper_motor_slave_register.h
 *
 *  Created on: Jun 24, 2018
 *      Author: robotic_tools
 */

#ifndef TRAJECTORY_MASTER_REGISTER_H_
#define TRAJECTORY_MASTER_REGISTER_H_

#include <robotic_tools/trajectory/register/trajectory_base_register.h>


class TrajectoryMasterRegister : public TrajectoryBaseRegister
{
public:

	TrajectoryMasterRegister();

	void set_register(Registers, uint32_t) override;
	uint32_t get_register(Registers) override;


	uint32_t get_buf_len();

private:
	uint32_t datareg_command = 0;
	uint32_t datareg_profile = 0;
	uint32_t datareg_param[16] = {0};
	uint32_t datareg_cpu_frequency = 0;
	uint32_t datareg_buf_len = 0;
	uint32_t datareg_buf_new = 0;
	uint32_t datareg_buf_add = 0;
};

#endif /* TRAJECTORY_MASTER_REGISTER_H_ */
