/*
 * trajectory.h
 *
 *  Created on: Apr 27, 2018
 *      Author: robotic_tools-ur
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_



#include <robotic_tools/trajectory/profile/profile_concave_scurve_vel.h>
#include <robotic_tools/trajectory/arduino/waveform_generation.h>



//#define TRAJECTORY_BUFFER_SIZE 256



class Trajectory
{
public:
	Trajectory(WaveformGeneration*, const uint32_t f_cpu);

	WaveformGeneration* wave_gen;
	const uint32_t f_cpu;

	enum class Profile : unsigned char
	{
		NONE,
		SCURVE_VEL,
		CONCAVE_SCURVE_VEL,
		TRAPEZOIDAL_VEL,
		CONSTANT_VEL,
		CUBIC_SPLINE_INTERPOLATION,
		QUINTIC_SPLINE_INTERPOLATION,
	};

	static const uint8_t MAX_PARAMETERS = 16;


	void run();
	uint16_t generate(Profile profile_mode, uint32_t params[]);


	uint32_t* get_params();
	void set_params(uint32_t idx, uint32_t data);

	uint16_t* const get_buf();
	void set_buf(uint32_t idx, uint16_t data);

	Profile profile_mode = Profile::NONE;


	enum class Direction : bool
	{
		CLOCKWISE = 0,
		COUNTER_CLOCKWISE = 1,
	};
	static const  uint16_t BUF_SIZE = 512;

private:
	/* Buffer for profile trajectory */
	uint16_t buf[BUF_SIZE];
	uint32_t params[MAX_PARAMETERS] = {0};



};


#endif /* TRAJECTORY_H_ */
