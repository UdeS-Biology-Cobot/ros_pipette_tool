/*
 * concave_scurve_vel_profile.h
 *
 *  Created on: May 22, 2018
 *      Author: robotic_tools
 */

#ifndef PROFILE_CONCAVE_SCURVE_VEL_H_
#define PROFILE_CONCAVE_SCURVE_VEL_H_


#include <stdint.h>


class ProfileConcaveScurveVel
{
public:
	ProfileConcaveScurveVel(uint16_t* const buf, uint16_t buf_len);


	uint16_t create(const uint32_t f_cpu, uint32_t steps, uint32_t f_min, uint32_t f_max, uint32_t slope);
	//void run();

private:


	uint16_t get_ptr_addr(uint16_t* p_addr);

	uint16_t* const buf;
	uint16_t buf_len;
};



#endif /* PROFILE_CONCAVE_SCURVE_VEL_H_ */
