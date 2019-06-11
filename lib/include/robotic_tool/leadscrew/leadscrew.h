/*
 * leadscrew.h
 *
 *  Created on: Jun 17, 2018
 *      Author: biobot
 */

#ifndef LEADSCREW_H_
#define LEADSCREW_H_


#include <stdint.h>

class Leadscrew
{
public:

	enum class Handedness : bool
	{
		LEFT_HANDED,
		RIGHT_HANDED,
	};

	Leadscrew(double lead, Handedness handedness);

	double lead;
	Handedness handedness;





};


#endif /* LEADSCREW_H_ */
