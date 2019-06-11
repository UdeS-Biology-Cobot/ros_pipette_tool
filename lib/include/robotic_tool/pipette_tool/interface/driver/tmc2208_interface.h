/*
 * tmc2208_interface.h
 *
 *  Created on: Dec 1, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_INTERFACE_DRIVER_ARDUINO_TMC2208_INTERFACE_H_
#define LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_INTERFACE_DRIVER_ARDUINO_TMC2208_INTERFACE_H_




#if defined(ARDUINO)
#include <robotic_tool/driver/arduino/tmc2208.h>



#include <robotic_tool/pipette_tool/interface/driver_interface.h>

class TMC2208Interface : public DriverInterface, public TMC2208
{
public:
	TMC2208Interface(UartSerial* port, uint32_t baudrate, uint32_t dir_pin, uint32_t en_pin, DriverInterfaceErr* p_err);



	uint32_t get_max_microstep(DriverInterfaceErr* p_err) override;
	uint32_t get_microstep(DriverInterfaceErr* p_err) override;
	void 	 set_microstep(uint32_t microstep, DriverInterfaceErr* p_err) override;

	void 	 set_direction(Direction dir, DriverInterfaceErr* p_err) override;

	void	 enable_motor(DriverInterfaceErr* p_err) override;
	void	 disable_motor(DriverInterfaceErr* p_err) override;




	bool err_get(DriverInterfaceErr* p_err) override;
	void err_clear(DriverInterfaceErr* p_err) override;

private:
		DriverInterfaceErr modify_err_code(TMC2208Err* p_err);
};

#endif	// ARDUINO

#endif /* LIB_BIOBOT_BIOBOT_PIPETTE_TOOL_INTERFACE_DRIVER_ARDUINO_TMC2208_INTERFACE_H_ */
