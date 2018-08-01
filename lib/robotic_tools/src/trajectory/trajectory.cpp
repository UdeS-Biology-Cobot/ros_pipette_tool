/*
 * trajectory.cpp
 *
 *  Created on: Apr 27, 2018
 *      Author: robotic_tools-ur
 */




#include <robotic_tools/trajectory/trajectory.h>
#include <robotic_tools/error_handle/error_handle.h>
//#include "trajectory.h"
//#include <error_handle.h>


#if defined(ARDUINO)

Trajectory::Trajectory(WaveformGeneration* wave_gen, const uint32_t f_cpu)
//:wave_gen(wave_gen), buf(buf), BUF_SIZE(buf_size)
:wave_gen(wave_gen), f_cpu(f_cpu)
{}


/* Trajectory already created in buffer*/
void Trajectory::run()
{
	wave_gen->run(buf, BUF_SIZE);
}




uint16_t Trajectory::generate(Profile profile_mode, uint32_t params[])
{
	this->profile_mode = profile_mode;

	uint16_t buf_len = 0;


	switch (profile_mode) {

		case Profile::CONCAVE_SCURVE_VEL:
		{
			ProfileConcaveScurveVel profile = ProfileConcaveScurveVel(buf, BUF_SIZE);

			/* Populate buffer with profile */
			buf_len = profile.create(f_cpu, params[0], params[1], params[2], params[3]);
			if(p_err->get()) {goto error;}
			break;
		}
		default:
			p_err->set(TrajectoryErrCode::TRAJECTORY_NOT_IMPLEMENTED);
			//p_err->set(TrajectoryErrCode::TRAJECTORY_BUFFER_OVF);

			goto error;
	}

	return buf_len;

error:
	return 0;
}

uint32_t* Trajectory::get_params()
{
	return params;
};


void Trajectory::set_params(uint32_t idx, uint32_t data)
{
	if (idx >= MAX_PARAMETERS) {
		p_err->set(TrajectoryErrCode::TRAJECTORY_PARAMS_OVF);
		return;
	}

	params[idx] = data;

};


uint16_t* const Trajectory::get_buf()
{
	return buf;
}

void Trajectory::set_buf(uint32_t idx, uint16_t data)
{
	if (idx >= BUF_SIZE) {
		p_err->set(TrajectoryErrCode::TRAJECTORY_BUFFER_OVF);
		return;
	}

	buf[idx] = data;
}

#endif
