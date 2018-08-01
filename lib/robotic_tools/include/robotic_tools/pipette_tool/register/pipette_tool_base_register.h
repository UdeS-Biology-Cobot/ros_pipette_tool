/*
 * pipette_tool_register.h
 *
 *  Created on: Jun 18, 2018
 *      Author: robotic_tools
 */

#ifndef PIPETTE_TOOL_BASE_REGISTER_H_
#define PIPETTE_TOOL_BASE_REGISTER_H_



#include <robotic_tools/pipette_tool/pipette_tool.h>

class PipetteToolBaseRegister
{
public:

	PipetteToolBaseRegister();





	enum class Registers {
		TYPE,
		SERIAL_NUMBER,
		DRIVER,

		SIZE	/* ALWAYS LAST  */
	};

	virtual void set_register(Registers, uint32_t) = 0;
	virtual uint32_t get_register(Registers) = 0;



};

#endif /* PIPETTE_TOOL_BASE_REGISTER_H_ */
