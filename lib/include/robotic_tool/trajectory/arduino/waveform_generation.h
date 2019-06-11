/*
 * waveform_generation.h
 *
 *  Created on: Jun 13, 2018
 *      Author: biobot
 */




/*
 * 	NOTE
 *
 * 	The pin selection is modular. By doing so, the maximum frequency of a pulse is
 * 	reduced from 580 kHz to 540 kHz.
 *
 *	Using PERB/WAVEB adds a signifiant delay before the output becomes active. (700 ms to 5 seconds)
 *	To counter this, initialize with PER but use PERB elsewhere.
 *	(https://forum.arduino.cc/index.php?topic=346731.0)
 */


#ifndef WAVEFORM_GENERATION_H_
#define WAVEFORM_GENERATION_H_


#include <stdint.h>


enum class WaveformGenerationErr : uint8_t {
	NONE,
	INVALID_PWM_PIN,
	PIN_MAPPING_FAILED,

	INVALID_FREQUENCY,
	MAX_FREQUENCY,
	INVALID_TOP,
	MAX_TOP,

	SIZE,
	FREQ_CONVERSION_NOT_DONE,
	FREQ_CONVERSION_ALREADY_DONE,
	PULSE_MISMATCH,


};


#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)


#include <Arduino.h>
#include <robotic_tool/trajectory/buffer_mode/buffer_mode.h>


class WaveformGeneration {
public:

	// Pin available for TCC0 (There are other availables pins not implemented)
	enum class PwmPin : uint8_t  {
		D0 = 0,
		D1 = 1,
		D2 = 2,
		D3 = 3,
		D4 = 4,
		D5 = 5,
		D6 = 6,
		D7 = 7,
		A3 = 18,
		A4 = 19,
	};

	WaveformGeneration(PwmPin, WaveformGenerationErr* p_err);
	~WaveformGeneration();

	void gen_dual_slope_pwm(bm_compact  bm,
							  uint32_t  buf[],
			     WaveformGenerationErr* p_err,
					 	          bool  disable_systick = true,
							      bool  disable_usbserial = true);

	uint32_t calculate_top_dual_slope_pwm(uint32_t freq, WaveformGenerationErr* p_err);

	void convert_frequencies(bm_compact*, uint32_t buf[], WaveformGenerationErr* p_err);

	static void stop_generation();


	uint32_t const MAX_FREQUENCY = 500000;	// Actually its 540 kHz but just to be safe

	//WaveformGenerationErr ec = WaveformGenerationErr::NONE;

	bool err_get(WaveformGenerationErr* p_err);
	void err_clear(WaveformGenerationErr* p_err);

private:
	PwmPin pwm_pin;
	RwReg& REG_COMPARE_CHANNEL_X;

	void stop_dual_slope_pwm(bool  disable_systick = true, bool  disable_usbserial = true);

	uint32_t CCX = 6;	// MUST be PAIR and low to be able to reach high frequencies
						// CCX = 6 EQUALS to 128 ns if TCC0 is equal to 48 MHz
						// TYPICAL TMC2208 step input high time = 100 ns

	const uint32_t F_GCLK4_TCC0 = 48000000;

	bool waveform_on = false;	// Waveform is being generated

	void map_pins(WaveformGenerationErr* p_err);
	RwReg& get_ccx(WaveformGenerationErr* p_err);

	void start_dual_slope_pwm(uint32_t top,
			                  uint16_t ccx,
							      bool disable_systick,
								  bool disable_usbserial);
};



#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO









#endif /* WAVEFORM_GENERATION_H_ */
