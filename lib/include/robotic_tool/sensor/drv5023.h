/*
 * drv5023.h
 *
 *  Created on: Nov 13, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_SENSOR_DRV5023_H_
#define LIB_BIOBOT_BIOBOT_SENSOR_DRV5023_H_


#if defined(ARDUINO)


#include <Arduino.h>


enum class DRV5023Err : uint8_t {
	NONE,
	DIGITAL_READ_ERROR
};



class DRV5023 {
public:

	enum class ErrorCode : uint8_t {
		NONE,
		DIGITAL_READ_ERROR
	};

	DRV5023(uint32_t trigger_pin);

	void enable_interrupt(voidFuncPtr callback, uint32_t mode);
	void disable_interrupt();

	bool is_triggered(DRV5023Err* p_err);	// If hall-effect sensor is LOW

	bool err_get(DRV5023Err* p_err);
	void err_clear(DRV5023Err* p_err);
private:
	uint32_t trigger_pin;
	bool is_enabled = false;

};



#endif // ARDUINO





#endif /* LIB_BIOBOT_BIOBOT_SENSOR_DRV5023_H_ */
