/*
 * protocol_trajectory.h
 *
 *  Created on: May 29, 2018
 *      Author: robotic_tools
 */

#ifndef TRAJECTORY_BASE_REGISTER_H_
#define TRAJECTORY_BASE_REGISTER_H_


#include <robotic_tools/trajectory/trajectory.h>

class TrajectoryBaseRegister
{
public:

	TrajectoryBaseRegister();

	//Trajectory* trajectory;

	//Trajectory* trajectory;

	enum class Registers : uint8_t
	{
		COMMAND,
		PROFILE,

		PARAM0,		/* On the fly trajectory */
		PARAM1,
		PARAM2,
		PARAM3,
		PARAM4,
		PARAM5,
		PARAM6,
		PARAM7,
		PARAM8,
		PARAM9,
		PARAM10,
		PARAM11,
		PARAM12,
		PARAM13,
		PARAM14,
		PARAM15,

		CPU_FREQUENCY,

		BUF_LEN,
		BUF_NEW,	/* Precalculated trajectory */
		BUF_ADD,

		SIZE		/* ALWAYS AT END */
	};

	/* Parameters based on trajectory	*/

	/*+---------+---------+---------+---------+---------+---------+---------+*/
	/*|    0    |    1    |    2    |    3    |    4    |    5    |    6    |*/
	/*+---------+---------+---------+---------+---------+---------+---------+*/
	/*| steps   |         |         |         |         |         |         |*/
	/*| f_min   |         |         |         |         |         |         |*/
	/*| f_max   |         |         |         |         |         |         |*/
	/*| slope   |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*|         |         |         |         |         |         |         |*/
	/*+---------+---------+---------+---------+---------+---------+---------+*/

	enum class ProfileBinary : uint8_t
	{
		NONE,
		SCURVE_VEL,
		CONCAVE_SCURVE_VEL,
		TRAPEZOIDAL_VEL,
		CONSTANT_VEL,
		CUBIC_SPLINE_INTERPOLATION,
		QUINTIC_SPLINE_INTERPOLATION,
	};









	// REGISTER GENERATE

	enum class Command : uint8_t
	{
		RUN_TRAJECTORY,
		GENERATE_WITH_PARAMS,
	};




	// REGISTER COMMAND
	static const uint8_t RUN_TRAJECTORY_BP			=  31;
	static const uint8_t GENERATE_WITH_PARAMS_BP	=  30;

	static const uint32_t RUN_TRAJECTORY_BM			=  0x80000000;
	static const uint32_t GENERATE_WITH_PARAMS_BM	=  0x40000000;












	virtual void set_register(Registers, uint32_t) = 0;
	virtual uint32_t get_register(Registers) = 0;



	Trajectory::Profile decode_profile(uint32_t data);
	bool decode_command(Command cmd, uint32_t);


	uint32_t encode_profile(Trajectory::Profile);
	uint32_t encode_command(Command cmd);




};












#endif /* TRAJECTORY_BASE_REGISTER_H_ */
