/*
 * profile.h
 *
 *  Created on: Nov 3, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_TRAJECTORY_MOTION_PROFILE_H_
#define LIB_BIOBOT_BIOBOT_TRAJECTORY_MOTION_PROFILE_H_


#include <robotic_tool/trajectory/buffer_mode/buffer_mode.h>



enum class MotionProfileErr : uint8_t {
	NONE,
	INVALID_PARAM,
	BUFFER_TOO_SMALL,
	VEL_MAX_EXCEEDED,
	TRAPEZOIDALE_WITHOUT_PLATEAU,
};


class MotionProfile {
public:





	MotionProfile();

	bm_compact compute_frequency_vel(uint32_t* buf,
									 uint32_t  buf_len,
							   	     uint32_t  steps,
									 uint32_t  f_min,
									 uint32_t  f_max,
									 uint32_t  slope,
							 MotionProfileErr* p_err);


	bm_compact compute_trapezoidal_vel(uint32_t* buf,
			 	 	 	 	 	 	   uint32_t  buf_len,
									   uint32_t  steps,
									     double  d_step,	// Distance per step (m)
										 double  v_max,		// Max velocity (m/s)
										 double  a_max,
							   MotionProfileErr* p_err);


	bool err_get(MotionProfileErr* p_err);
	void err_clear(MotionProfileErr* p_err);

};








#endif /* LIB_BIOBOT_BIOBOT_TRAJECTORY_PROFILE_PROFILE_H_ */
