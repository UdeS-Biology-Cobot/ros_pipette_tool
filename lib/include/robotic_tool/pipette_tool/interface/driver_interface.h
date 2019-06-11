/*
 * driver_interface.h
 *
 *  Created on: Nov 30, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_INTERFACE_DRIVER_INTERFACE_H_
#define LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_INTERFACE_DRIVER_INTERFACE_H_


#include <stdint.h>

// Error Code for all encoders
enum class DriverInterfaceErr : uint32_t {
	NONE,

	TIMEOUT,
	INTERNAL_ERROR,
};



//class AMT21Abstract


class DriverInterface
{
public:
	//EncoderInterface();

	enum class Direction : bool {
		CW = 0,			// Stepper Channel A = Sine, B = Cosine
		CCW = 1,
	};


	virtual ~DriverInterface() {};

	virtual uint32_t get_max_microstep(DriverInterfaceErr* p_err) = 0;
	virtual uint32_t get_microstep(DriverInterfaceErr* p_err) = 0;
	virtual void 	 set_microstep(uint32_t microstep, DriverInterfaceErr* p_err) = 0;

	virtual void 	 set_direction(Direction dir, DriverInterfaceErr* p_err) = 0;

	virtual void	 enable_motor(DriverInterfaceErr* p_err) = 0;
	virtual void	 disable_motor(DriverInterfaceErr* p_err) = 0;



	virtual bool err_get(DriverInterfaceErr* p_err) = 0;
	virtual void err_clear(DriverInterfaceErr* p_err) = 0;
};






#endif /* LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_INTERFACE_DRIVER_INTERFACE_H_ */
