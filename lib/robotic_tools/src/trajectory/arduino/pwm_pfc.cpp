/*
 * hw.cpp
 *
 *  Created on: Apr 27, 2018
 *      Author: robotic_tools-ur
 */

//#include "pwm_pfc.h"


/* TODO remove FCPU from define */
//#define F_CPU 16000000

#include <robotic_tools/trajectory/arduino/pwm_pfc.h>
#include <robotic_tools/error_handle/error_handle.h>


uint16_t PwmPfc::calculate_top (uint32_t freq, const uint32_t f_cpu) {
	/*TODO add prescaling N factor */
	uint32_t top = f_cpu / (2 * freq);

	if (top >= 65536) {
		p_err->set(PwmPfcErrCode::INVALID_FREQUENCY_FOR_TOP_CALCULATION);
	}

	return top;
}


#if defined(ARDUINO)





#include "avr/interrupt.h"
#include "USBAPI.h"

//#include <error_handle.h>


volatile bool TIMER1_OVF_FLAG = false;


PwmPfc::PwmPfc()
{
	//pinMode(OC1B_PIN, OUTPUT);	/* Set OC1B pin */
}


PwmPfc::~PwmPfc()
{
	//pinMode(OC1B_PIN, INPUT);
	stop();
}


void PwmPfc::start(uint16_t top, uint16_t ocr) {
	/* Stop interrupts */
	cli();

	/* Disable Timer 0, Delay functions wont work anymore	*/
	TCCR0A = 0;
	TCCR0B = 0;
	TIMSK0 = 0;

	/* Disable USB 											*/
	USBINT = 0;
	USBSTA = 0;
	USBCON = 0;


	/* PHASE and Frequency Correct Mode Timer 1 													*/
	/* Timer/Counter1 Control Register A */
	TCCR1A = 0; 				/* Set entire register to 0 										*/
	TCCR1A |= (1 << COM1B0); 	/* Set OC1B on compare match when up-counting. Clear OC1B on ...	*/
	TCCR1A |= (1 << COM1B1);	/* ... compare match when down-counting.							*/

	TCCR1A |= (1 << WGM10); 	/* Set Timer/Counter mode to PWM, Phase and Frequency Correct		*/

	/* Timer/Counter1 Control Register B */
	TCCR1B = 0; 				/* Set entire register to 0 										*/
	TCCR1B |= (1 << CS10);		/* Clock select clk_IO/1 (No prescaling)							*/
	TCCR1B |= (1 << WGM13); 	/* Set Timer/Counter mode to PWM, Phase and Frequency Correct		*/

	TCNT1 = 0; 					/* Initialize counter value to 0									*/
	OCR1A = top; 				/* Set initial TOP value for compare match register					*/
	OCR1B = ocr;

	TIMSK1 |= (1 << TOIE1); 	/* Timer/Counter1, Overflow Interrupt Enable						*/

	/* Enable interrupts */
	sei();
}


void PwmPfc::stop() {
	/* Stop interrupts */
	cli();
	/* No clock source. (Timer/Counter stopped) */
	TCCR1B &= ~(1 << CS10);
	TCCR1B &= ~(1 << CS11);
	TCCR1B &= ~(1 << CS12);

	TIMER1_OVF_FLAG = false;

	/* Enable interrupts */
	sei();

	/* Restore configuration for Timer0 and USB */
	init();
	USBDevice.attach();
}


void PwmPfc::set_top(uint16_t top) {
	OCR1A = top;
}


void PwmPfc::set_ocr(uint16_t ocr) {
	OCR1B  = ocr;
}





volatile bool PwmPfc::get_isr_flag() {
	return TIMER1_OVF_FLAG;
}


void PwmPfc::set_isr_flag(bool flag) {
	TIMER1_OVF_FLAG = flag;
}



/* TIMER1 OVERFLOW INTERRUPT */
ISR(TIMER1_OVF_vect){
	TIMER1_OVF_FLAG = true;
}






void PwmPfc::run(uint16_t const buf[], uint16_t buf_len) {

	uint8_t mode;
	uint16_t loop_freq;
	uint32_t loop_ctr = 0;
	uint32_t loop_ctr_max;
	uint16_t loop_ctr_max_high;
	uint16_t loop_ctr_max_low;
	uint16_t top;
	uint16_t ocr = 40;


	uint16_t buf_ctr = 0;

	RUNNING_TRAJECTORY_FLAG  =  true;
	UPDATE_TRAJECTORY_FLAG   =  false;

	STOP_PWM_PFC_FLAG 		  =  false;
	STOP_TRAJECTORY_FLAG 	  =  false;
	PRE_STOP_TRAJECTORY_FLAG  =  false;

	mode = buf[buf_ctr++];
	if (mode != BEGIN_END_MODE) {
		p_err->set(PwmPfcErrCode::BUFFER_WRONG_START_MODE);
		return;
	}

	/* TODO check if reverse pointer and if is in a valid range and MUST be somewhere where it s terminate to 0 */
	/* TODO check thas ending has a BEGIN_END_MODE*/

	mode = buf[buf_ctr++];

	if (mode == REVERSE_MODE) {
		p_err->set(PwmPfcErrCode::BUFFER_REVERSE_MODE_WRONG_USE);
		return;
	}
	else if (mode == LOOP_MODE) {
		top = buf[buf_ctr++];
		loop_freq = top;
		loop_ctr = 0;
		loop_ctr_max_high = buf[buf_ctr++];
		loop_ctr_max_low = buf[buf_ctr++];
		loop_ctr_max = ((uint32_t)loop_ctr_max_high) << 16 | loop_ctr_max_low;
	}
	else if (mode == NORMAL_MODE) {
		top = buf[buf_ctr++];
		if (buf[buf_ctr] == BEGIN_END_MODE) {
			PRE_STOP_TRAJECTORY_FLAG = true;
		}
	}
	else {
		p_err->set(PwmPfcErrCode::BUFFER_MODE_NOT_IMPLEMENTED);
		return;
	}

	start(top, ocr);

	while (RUNNING_TRAJECTORY_FLAG) {

		UPDATE_TRAJECTORY_FLAG = get_isr_flag();

		if (UPDATE_TRAJECTORY_FLAG) {
			set_top(top);
			set_isr_flag(false);

			if (STOP_TRAJECTORY_FLAG) {

				set_ocr(65535);		/* This stops the pwm pulses on the output compare pin on the ... 	*/
									/* ... next interrupt so that it can safely stop the interrupts  	*/
				if (STOP_PWM_PFC_FLAG) {
					stop();
					RUNNING_TRAJECTORY_FLAG = false;
				}
				STOP_PWM_PFC_FLAG = true;
			}
			else if (PRE_STOP_TRAJECTORY_FLAG) {
				STOP_TRAJECTORY_FLAG = true;
			}
			else {
				if (mode == LOOP_MODE) {
					top = loop_freq;
					loop_ctr++;

					/* When loop is finished */
					if (loop_ctr == loop_ctr_max) {
						if (buf[buf_ctr] == NORMAL_MODE) {
							buf_ctr++;
							mode = NORMAL_MODE;
						}
						else if (buf[buf_ctr] == REVERSE_MODE) {
							buf_ctr++;
							buf_ctr = buf[buf_ctr]; /* Change pointer to the address for decrementing */
							mode = REVERSE_MODE;
						}
						else if (buf[buf_ctr] == BEGIN_END_MODE) {
							STOP_TRAJECTORY_FLAG = true;	/* MUST stop on next interrupt */
						}
					}

				}
				else if (mode == REVERSE_MODE) {
					top = buf[buf_ctr--];

					/* TODO check for NORMAL an REVERSE MODE */

					/* Stop Fast Pwm if finishing in this mode */
					if (buf[buf_ctr] == NORMAL_MODE) {
						buf_ctr--;
						if (buf[buf_ctr] == BEGIN_END_MODE) {
							PRE_STOP_TRAJECTORY_FLAG = true; /* MUST stop on second next interrupt */
						}
					}
				}
				else if (mode == NORMAL_MODE) {
					top = buf[buf_ctr++];

					if (buf[buf_ctr] == REVERSE_MODE) {
						mode = REVERSE_MODE;
						buf_ctr++;
						buf_ctr = buf[buf_ctr]; /* Change pointer to the address for decrementing */
					}
					else if (buf[buf_ctr] == LOOP_MODE) {
						buf_ctr++;

						loop_freq = buf[buf_ctr++];
						loop_ctr_max_high = buf[buf_ctr++];
						loop_ctr_max_low = buf[buf_ctr++];

						loop_ctr_max = ((uint32_t)loop_ctr_max_high) << 16 | loop_ctr_max_low;
						loop_ctr = 0;
						mode = LOOP_MODE;
					}
					else if (buf[buf_ctr] == BEGIN_END_MODE) {
						PRE_STOP_TRAJECTORY_FLAG = true;
					}
				}
			}
		}
	}
}










uint8_t PwmPfc::get_output_pin()
{
	return OC1B_PIN;
}

#endif
