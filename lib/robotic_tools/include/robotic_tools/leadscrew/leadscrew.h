/*
 * leadscrew.h
 *
 *  Created on: Jun 17, 2018
 *      Author: robotic_tools
 */

#ifndef LEADSCREW_H_
#define LEADSCREW_H_


#include <stdint.h>

class Leadscrew
{
public:

	enum class ThreadDir : bool
	{
		LEFT_HAND,
		RIGHT_HAND,
	};

	Leadscrew(ThreadDir thread_dir, float lead);

	ThreadDir thread_dir;
	float lead;




};


#endif /* LEADSCREW_H_ */
