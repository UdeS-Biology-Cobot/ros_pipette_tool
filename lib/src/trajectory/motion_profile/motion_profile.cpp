/*
 * profile.cpp
 *
 *  Created on: Nov 3, 2018
 *      Author: biobot
 */


#include <robotic_tool/trajectory/motion_profile/motion_profile.h>
#include <math.h>


MotionProfile::MotionProfile()
{}

bm_compact MotionProfile::compute_frequency_vel(uint32_t* buf,
								 	 	        uint32_t  buf_len,
												uint32_t  steps,
												uint32_t  f_min,
												uint32_t  f_max,
												uint32_t  slope,
										MotionProfileErr* p_err)
{
	bm_compact bm;

	uint32_t slope_len = ceil((double)(f_max - f_min) / (double)slope);
	int32_t ramp_mode = steps - 2 * slope_len;	// NEGATIVE and ZERO --> Triangle profile
	uint32_t plateau_len = ramp_mode;

	bm.steps = steps;

	// Error checking
	if (f_max < f_min) {
		*p_err = MotionProfileErr::INVALID_PARAM;
		return bm;
	}

	// Generate ONE pulse, no buffer needed
	if (steps == 1) {
		bm.mode = BmCompactType::SINGLE;
		bm.plateau_freq = f_min;
		bm.plateau_len = steps;
		bm.buf_size = 0;
	}
	// Generate constant profile, no buffer needed
	else if (f_min == f_max){
		bm.mode = BmCompactType::CONSTANT;
		bm.plateau_freq = f_min;
		bm.plateau_len = steps;
		bm.buf_size = 0;
	}
	// Generate triangle profile
	else if (ramp_mode <= 0) {
		uint32_t freq;
		uint32_t ctr;
		uint32_t rampup_len = ceil(double(steps) / 2.0);
		uint32_t rampdown_len = steps - rampup_len;


		// Check if buffer has enough space
		if (buf_len < rampup_len) {
			*p_err = MotionProfileErr::BUFFER_TOO_SMALL;
			return bm;
		}

		// RAMPUP
		for (ctr = 0; ctr < rampup_len; ctr++) {
			freq = ((ctr*slope) + f_min);
			buf[ctr] = freq;
		}

		bm.mode = BmCompactType::RAMPUP_RAMPDOWN;
		bm.rampup_len = rampup_len;
		bm.rampdown_idx = rampdown_len - 1;
		bm.buf_size = rampup_len;
	}
	// Generate concave s-curve profile
	else {
		uint32_t freq;
		uint32_t ctr;
		uint32_t rampup_len = slope_len;
		uint32_t rampdown_len = rampup_len;

		// Check if buffer has enough space
		if (buf_len < rampup_len ) {
			*p_err = MotionProfileErr::BUFFER_TOO_SMALL;
			return bm;
		}

		// RAMP UP
		for (ctr = 0; ctr < rampup_len; ctr++) {
			freq = ((ctr*slope) + f_min);
			buf[ctr] = freq;
		}

		bm.mode = BmCompactType::FULL;
		bm.rampup_len = rampup_len;
		bm.rampdown_idx = rampdown_len - 1;
		bm.plateau_freq = f_max;
		bm.plateau_len = plateau_len;
		bm.buf_size = rampup_len;
	}

	return bm;
}


bm_compact MotionProfile::compute_trapezoidal_vel(uint32_t* buf,
		 	 	 	 	 	 	   	   	    uint32_t  buf_len,
											uint32_t  steps,
											  double  d_step,	// Distance per step (m)
											  double  v_max,	// Max velocity (m/s)
											  double  a_max,
									MotionProfileErr* p_err)
{
	bm_compact bm;

	bm.steps = steps;

	double t;		// Time (s)
	double t_c;		// Time for reaching max velocity  (s)
	double t_f;		// Total time of the trajectory (s)

	double p_i = 0; 				// Initial position (m)
	double p_f = d_step * steps;	// Final position (m)
	// d_step -> y
	double vel_at_max_acc = d_step / sqrt((d_step-p_i)/(0.5*a_max));	// Velocity @ Max acceleration

	t_f = (-a_max*p_i + a_max*p_f + v_max*v_max) / (a_max*v_max);
	t_c = (p_i - p_f + v_max*t_f) / v_max;



	// Error checking
	if (vel_at_max_acc > v_max) {
		*p_err = MotionProfileErr::VEL_MAX_EXCEEDED;
		return bm;
	}


	// Generate ONE pulse, no buffer needed
	if (steps == 1) {
		t = sqrt((d_step - p_i)/(0.5*a_max));

		bm.mode = BmCompactType::SINGLE;
		bm.plateau_freq = round(1/t);
		bm.plateau_len = steps;
		bm.buf_size = 0;
	}
	// Generate constant profile, no buffer needed
	else if (a_max == 0){
		t = d_step / v_max;

		bm.mode = BmCompactType::CONSTANT;
		bm.plateau_freq = floor(1/t);
		bm.plateau_len = steps;
		bm.buf_size = 0;
	}
	// Generate triangle profile
	else if (2*t_c >= t_f) {
		double acc_d_step = 0;
		double prev_t = 0;
		uint32_t freq;
		uint32_t step_ctr;
		uint32_t rampup_len = ceil(steps/2);
		uint32_t rampdown_len = steps - rampup_len;

		// Check if buffer has enough space
		if (buf_len < rampup_len) {
			*p_err = MotionProfileErr::BUFFER_TOO_SMALL;
			return bm;
		}

		for (step_ctr = 0; step_ctr < rampup_len; step_ctr++) {
			acc_d_step += d_step;
			t = sqrt((acc_d_step-p_i)/(0.5*a_max));
			freq = round(1.0/ (t - prev_t));

			buf[step_ctr] = freq;

			prev_t = t;
		}

		bm.mode = BmCompactType::RAMPUP_RAMPDOWN;
		bm.rampup_len = rampup_len;
		bm.rampdown_idx = rampdown_len - 1;
		bm.buf_size = rampup_len;
	}
	// Generate trapezoidale profile
	else {
		double acc_d_step = 0;
		double prev_t = 0;
		uint32_t freq;
		uint32_t step_ctr;
		uint32_t rampup_len;
		uint32_t rampdown_len;
		uint32_t plateau_len;
		//Serial.println("YOLO");

		for (step_ctr = 0; step_ctr < steps; step_ctr++) {
			acc_d_step += d_step;
			t = sqrt((acc_d_step-p_i)/(0.5*a_max));

			//Serial.print("diff_t = ");
			//Serial.println(t - prev_t,10);

			if (t > t_c) {
				break;
			}

			// Check if buffer has enough space
			if (step_ctr >= buf_len ) {
				*p_err = MotionProfileErr::BUFFER_TOO_SMALL;
				return bm;
			}
			freq = round(1.0 / (t - prev_t));
			buf[step_ctr] = freq;
			prev_t = t;
		}

		t = ((acc_d_step - p_i) / (a_max*t_c)) + t_c / 2;
		freq = round(1.0 / (t - prev_t));
		rampup_len = step_ctr;
		rampdown_len = rampup_len;
		plateau_len = steps - (rampup_len + rampdown_len);

		// TODO maybe not necessary
		if (plateau_len == 0) {
			*p_err = MotionProfileErr::TRAPEZOIDALE_WITHOUT_PLATEAU;
			return bm;
		}

		bm.mode = BmCompactType::FULL;
		bm.rampup_len = rampup_len;
		bm.rampdown_idx = rampdown_len - 1;
		bm.plateau_freq = freq;
		bm.plateau_len = plateau_len;
		bm.buf_size = rampup_len;
	}

	return bm;
}

bool MotionProfile::err_get(MotionProfileErr* p_err)
{
	if (*p_err == MotionProfileErr::NONE) {
		return false;
	}
	return true;
}

void MotionProfile::err_clear(MotionProfileErr* p_err)
{
	*p_err = MotionProfileErr::NONE;
}


