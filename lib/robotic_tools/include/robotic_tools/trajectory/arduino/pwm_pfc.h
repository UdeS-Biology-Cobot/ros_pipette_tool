/*
 * hw.h
 *
 *  Created on: Apr 27, 2018
 *      Author: robotic_tools-ur
 */

#ifndef PWM_PFC_H_
#define PWM_PFC_H_


#include <stdint.h>


/* PWM phase and frequency correct */
class PwmPfc {
public:

	PwmPfc();
	~PwmPfc();

	void run(uint16_t const buf[], uint16_t buf_len);
	uint8_t get_output_pin();

	static uint16_t calculate_top(uint32_t freq, const uint32_t f_cpu);


	enum Mode {
		BEGIN_END_MODE,
		REVERSE_MODE,
		LOOP_MODE,
		NORMAL_MODE
	};

	static uint32_t cpu_allo;
private:

	void set_top(uint16_t top);
	void set_ocr(uint16_t ocr);




	/*TODO add resolution calculation */

	volatile bool get_isr_flag();
	void set_isr_flag(bool flag);




	volatile bool UPDATE_TRAJECTORY_FLAG = false;

	bool STOP_PWM_PFC_FLAG 			=  false;
	bool STOP_TRAJECTORY_FLAG 		=  false;
	bool PRE_STOP_TRAJECTORY_FLAG 	=  false;
	bool RUNNING_TRAJECTORY_FLAG 	=  false;







	static const uint8_t OC1B_PIN = 10;

	void start(uint16_t top, uint16_t ocr);
	void stop();
};












#endif /* PWM_PFC_H_ */
