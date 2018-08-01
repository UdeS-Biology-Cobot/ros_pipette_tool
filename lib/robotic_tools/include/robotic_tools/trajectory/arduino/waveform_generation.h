/*
 * waveform_generation.h
 *
 *  Created on: Jun 13, 2018
 *      Author: robotic_tools
 */

#ifndef WAVEFORM_GENERATION_H_
#define WAVEFORM_GENERATION_H_


#include <stdint.h>


#include <robotic_tools/trajectory/arduino/pwm_pfc.h>


class WaveformGeneration {
private:
	void* obj;

public:





	WaveformGeneration(PwmPfc*);


	void run(uint16_t* const p_buf, uint16_t buf_size);
	uint8_t get_waveform_pin();

private:
	enum class Mode : uint8_t
	{
		PWM_PHASE_FREQUENCY_CORRECT,
	};

	Mode mode;




};













#endif /* WAVEFORM_GENERATION_H_ */
