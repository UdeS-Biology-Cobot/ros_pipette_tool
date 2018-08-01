/*
 * protocol_trajectory.cpp
 *
 *  Created on: May 29, 2018
 *      Author: robotic_tools
 */





#include <robotic_tools/error_handle/error_handle.h>
#include <robotic_tools/trajectory/register/trajectory_base_register.h>



TrajectoryBaseRegister::TrajectoryBaseRegister()
{}



Trajectory::Profile TrajectoryBaseRegister::decode_profile(uint32_t data)
{
	ProfileBinary profile_bin = ProfileBinary(data);
	Trajectory::Profile profile;

	switch (profile_bin)
	{
		case ProfileBinary::CONCAVE_SCURVE_VEL:
			profile = Trajectory::Profile::CONCAVE_SCURVE_VEL;
			break;

		default:
			p_err->set(TrajectoryRegisterErrCode::DECODE_PROFILE_NOT_IMPLEMENTED);
	}

	return profile;
}


uint32_t TrajectoryBaseRegister::encode_profile(Trajectory::Profile profile)
{
	ProfileBinary profile_bin;

	switch (profile)
	{
		case Trajectory::Profile::CONCAVE_SCURVE_VEL:
			profile_bin = ProfileBinary::CONCAVE_SCURVE_VEL;
			break;

		case Trajectory::Profile::NONE:
			profile_bin = ProfileBinary::NONE;
			break;


		default:
			p_err->set(TrajectoryRegisterErrCode::ENCODE_PROFILE_NOT_IMPLEMENTED);
			break;
	}

	return (uint32_t)profile_bin;
}


bool TrajectoryBaseRegister::decode_command(Command cmd, uint32_t data)
{
	bool rc;
	switch (cmd) {
		case Command::RUN_TRAJECTORY:
			rc = (data & RUN_TRAJECTORY_BM) >> RUN_TRAJECTORY_BP;
			break;

		case Command::GENERATE_WITH_PARAMS:
			rc = (data & GENERATE_WITH_PARAMS_BM) >> GENERATE_WITH_PARAMS_BP;
			break;

		default:
			/* TODO error */
			break;
	}
	return rc;
}

uint32_t TrajectoryBaseRegister::encode_command(Command cmd)
{
	uint32_t rc = 0;

	switch (cmd) {
		case Command::RUN_TRAJECTORY:
			rc =  (uint32_t)1 << RUN_TRAJECTORY_BP;
			break;

		case Command::GENERATE_WITH_PARAMS:
			rc =  (uint32_t)1 << GENERATE_WITH_PARAMS_BP;
			break;

		default:
			/* TODO error */
			break;
	}
	return rc;
}












