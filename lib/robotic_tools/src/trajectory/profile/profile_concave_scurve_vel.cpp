/*
 * concave_scurve_vel_profile.cpp
 *
 *  Created on: May 22, 2018
 *      Author: robotic_tools
 */





#include <robotic_tools/trajectory/profile/profile_concave_scurve_vel.h>


#include <math.h>

#include <robotic_tools/error_handle/error_handle.h>
#include <robotic_tools/trajectory/arduino/pwm_pfc.h>


ProfileConcaveScurveVel::ProfileConcaveScurveVel(uint16_t* const buf, uint16_t buf_len)
:buf(buf), buf_len(buf_len)
{}




uint16_t ProfileConcaveScurveVel::create(const uint32_t f_cpu, uint32_t steps, uint32_t f_min, uint32_t f_max, uint32_t slope) {

	//uint16_t *p_buf = p_buf_head;


	uint32_t slope_len = (f_max - f_min) / slope;
	/* TODO check if slope_len * 2 dosent exceed signed 32 bit */
	int32_t ramp_mode = steps - 2 * slope_len;	/* NEGATIVE and ZERO --> Triangle profile			*/
											/* GREATER THEN ZERO --> Concave s-curve profile	*/
	uint32_t plateau_len= steps - 2 * slope_len;
	uint16_t plateau_len_high;
	uint16_t plateau_len_low;


	uint32_t i;
	uint32_t freq;
	uint16_t top;
	uint16_t addr_pointer;
	uint32_t rampup_len;
	uint32_t rampdown_len;




	uint16_t buf_ctr = 0;

	/* Error checking	*/
	if (f_max < f_min) {
		p_err->set(ProfileConcaveScurveVelErrCode::PROFILE_CREATION_BAD_PARAMETERS);
		goto error;
	}

	/* Generate ONE pulse */
	if (steps == 1) {

		/* Check if buffer has enough space */
		if (buf_len < 4) {
			p_err->set(ProfileConcaveScurveVelErrCode::PROFILE_CREATION_BUFFER_OVERFLOW);
			goto error;
		}

		buf[buf_ctr++] = PwmPfc::BEGIN_END_MODE;
		buf[buf_ctr++] = PwmPfc::NORMAL_MODE;
		freq = f_min;
		top = PwmPfc::calculate_top(freq, f_cpu);
		if(p_err->get()) {goto error;}

		buf[buf_ctr++] = top;
		buf[buf_ctr++] = PwmPfc::BEGIN_END_MODE;
	}
	/* Generate constant profile */
	else if (f_min == f_max){

		/* Check if buffer has enough space */
		if (buf_len < 6) {
			p_err->set(ProfileConcaveScurveVelErrCode::PROFILE_CREATION_BUFFER_OVERFLOW);
			goto error;
		}

		/* BEGIN MODE */
		buf[buf_ctr++] = PwmPfc::BEGIN_END_MODE;

		/* CONSTANT RAMP 					*/
		buf[buf_ctr++] = PwmPfc::LOOP_MODE;
		top = PwmPfc::calculate_top(f_min, f_cpu);
		if(p_err->get()) {goto error;}
		buf[buf_ctr++] = top;				/* Frequency 		*/

		plateau_len_high = (uint32_t)(steps >> 16);
		plateau_len_low = (uint32_t) steps;

		buf[buf_ctr++] = plateau_len_high;		/* Loop N times 	*/
		buf[buf_ctr++] = plateau_len_low;

		/* BEGIN MODE */
		buf[buf_ctr++] = PwmPfc::BEGIN_END_MODE;

	}
	/* Generate triangle profile */
	else if (ramp_mode <= 0) {

		rampup_len = ceil(double(steps) / 2.0);
		rampdown_len = steps - rampup_len;

		/* Check if buffer has enough space */
		if (buf_len < (2 + rampup_len + 3)) {
			p_err->set(ProfileConcaveScurveVelErrCode::PROFILE_CREATION_BUFFER_OVERFLOW);
			goto error;
		}

		/* BEGIN MODE */
		buf[buf_ctr++] = PwmPfc::BEGIN_END_MODE;

		/* NORMAL MODE */
		buf[buf_ctr++] = PwmPfc::NORMAL_MODE;

		/* RAMP UP */
		for (i = 0; i < rampup_len; i++) {

			freq = ((i*slope) + f_min);
			top = PwmPfc::calculate_top(freq, f_cpu);
			if(p_err->get()) {goto error;}

			if (i == rampdown_len - 1) {
				addr_pointer = buf_ctr;
			}
			buf[buf_ctr++] = top;
		}

		/* RAMP DOWN */
		buf[buf_ctr++] = PwmPfc::REVERSE_MODE;
		buf[buf_ctr++] = addr_pointer;

		buf[buf_ctr++] = PwmPfc::BEGIN_END_MODE;
	}
	/* Generate concave s-curve profile */
	else {

		rampup_len = slope_len;
		rampdown_len = rampup_len;

		/* Check if buffer has enough space */
		if (buf_len < (2 + rampup_len + 4 + 3)) {
			p_err->set(ProfileConcaveScurveVelErrCode::PROFILE_CREATION_BUFFER_OVERFLOW);
			goto error;
		}

		/* BEGIN MODE */
		buf[buf_ctr++] = PwmPfc::BEGIN_END_MODE;

		/* NORMAL MODE */
		buf[buf_ctr++] = PwmPfc::NORMAL_MODE;

		/* RAMP UP */
		for (i = 0; i < rampup_len; i++) {

			freq = ((i*slope) + f_min);
			top = PwmPfc::calculate_top(freq, f_cpu);
			if(p_err->get()) {goto error;}


			if (i == rampdown_len - 1) {
				addr_pointer = buf_ctr;
			}

			buf[buf_ctr++] = top;
		}

		/* CONSTANT RAMP 					*/
		buf[buf_ctr++] = PwmPfc::LOOP_MODE;
		top = PwmPfc::calculate_top(f_max, f_cpu);
		if(p_err->get()) {goto error;}
		buf[buf_ctr++] = top;				/* Frequency 		*/

		plateau_len_high = (uint32_t)(plateau_len >> 16);
		plateau_len_low = (uint32_t) plateau_len;

		buf[buf_ctr++] = plateau_len_high;		/* Loop N times 	*/
		buf[buf_ctr++] = plateau_len_low;


		/* RAMP DOWN */
		buf[buf_ctr++] = PwmPfc::REVERSE_MODE;
		buf[buf_ctr++] = addr_pointer;

		buf[buf_ctr++] = PwmPfc::BEGIN_END_MODE;
	}

	return buf_ctr;

error:
	return 0;
}

