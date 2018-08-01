/*
 * drv8880_base_register.h
 *
 *  Created on: Jun 22, 2018
 *      Author: robotic_tools
 */

#ifndef DRV8880_BASE_REGISTER_H_
#define DRV8880_BASE_REGISTER_H_


#include <robotic_tools/stepper_driver/arduino/drv8880.h>

class DRV8880BaseRegister
{
public:

	DRV8880BaseRegister();
	virtual ~DRV8880BaseRegister();
	enum class Registers {
		CONFIG,
		MICROSTEP,

		SIZE	/* ALWAYS LAST */
	};


	enum class MicrostepModeBinary: unsigned short
	{
		FULL_STEP = 0,
		HALF_STEP,
		HALF_STEP_CIRCULAR,
		QUARTER_STEP,
		EIGHTH_STEP,
		SIXTEENTH_STEP,
	};

	enum class CurrentBinary : uint8_t
	{
		PERCENT_25 = 0,
		PERCENT_50,
		PERCENT_75,
		PERCENT_100,
	};

	enum class DirectionBinary : bool
	{
		CLOCKWISE = 0,
		COUNTER_CLOCKWISE = 1,
	};

/*
	enum class DataRegDRV8880 : uint8_t
	{
		MRES,
		CURRENT,
		DIRECTION,
		ENABLE,
		SLEEP,
		FAULT,
	};
*/

	//uint16_t decode_microsteps(DRV8880::Mres);
	//uint16_t decode_microstep(uint32_t microstep);
	DRV8880::MicrostepMode decode_microstep_mode(uint32_t data);
	DRV8880::Current decode_current(uint32_t data);
	DRV8880::Direction decode_direction(uint32_t data);



	//uint32_t encode_microstep(uint16_t microstep);
	uint32_t encode_microstep_mode(DRV8880::MicrostepMode);
	uint32_t encode_current(DRV8880::Current);
	uint32_t encode_direction(DRV8880::Direction);

	uint32_t encode_microstep_mode(DRV8880::MicrostepMode, uint32_t data);
	uint32_t encode_current(DRV8880::Current, uint32_t data);
	uint32_t encode_direction(DRV8880::Direction, uint32_t data);


	virtual void set_register(Registers, uint32_t) = 0;
	virtual uint32_t get_register(Registers) = 0;

protected:

	// REGISTER_CONFIG
	static const uint8_t MRES_BP				=  28;
	static const uint8_t CURRENT_BP				=  26;
	static const uint8_t DIRECTION_BP			=  25;
	static const uint8_t ENABLE_BP				=  24;
	static const uint8_t SLEEP_BP				=  23;
	static const uint8_t FAULT_BP				=  22;



	static const uint32_t MRES_BM				=  0xF0000000;
	static const uint32_t CURRENT_BM			=  0x0C000000;
	static const uint32_t DIRECTION_BM			=  0x02000000;
	static const uint32_t ENABLE_BM				=  0x01000000;
	static const uint32_t SLEEP_BM				=  0x00800000;
	static const uint32_t FAULT_BM				=  0x00400000;
};


#endif /* DRV8880_BASE_REGISTER_H_ */
