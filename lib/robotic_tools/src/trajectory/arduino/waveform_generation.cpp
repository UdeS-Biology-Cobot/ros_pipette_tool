/*
 * waveform_generation.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: robotic_tools
 */

#if defined(ARDUINO)

#include <robotic_tools/trajectory/arduino/waveform_generation.h>



WaveformGeneration::WaveformGeneration(PwmPfc* obj)
:obj(obj), mode(Mode::PWM_PHASE_FREQUENCY_CORRECT)
{}


void WaveformGeneration::run(uint16_t* const p_buf, uint16_t buf_size)
{
	switch(mode) {

		case Mode::PWM_PHASE_FREQUENCY_CORRECT:
			((PwmPfc*)obj)->run(p_buf, buf_size);
			break;


	};
}


uint8_t WaveformGeneration::get_waveform_pin()
{
	switch(mode) {

		case Mode::PWM_PHASE_FREQUENCY_CORRECT:
			return ((PwmPfc*)obj)->get_output_pin();

	};
	/* TODO add error*/
	return 0;
}

#endif
