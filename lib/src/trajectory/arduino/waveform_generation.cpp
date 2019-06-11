/*
 * waveform_generation.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: biobot
 */

#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)

#include <robotic_tool/trajectory/arduino/waveform_generation.h>










/*
 *	Macros for disabling and restoring interrupts
 *
 * 	- https://mcuoneclipse.com/2014/01/26/entercritical-and-exitcritical-why-things-are-failing-badly/
 */
#define CpuCriticalVar()  uint8_t cpuSR

#define CpuEnterCritical()              \
  do {                                  \
    asm (                               \
    "MRS   R0, PRIMASK\n\t"             \
    "CPSID I\n\t"                       \
    "STRB R0, %[output]"                \
    : [output] "=m" (cpuSR) :: "r0");   \
  } while(0)

#define CpuExitCritical()               \
  do{                                   \
    asm (                               \
    "ldrb r0, %[input]\n\t"             \
    "msr PRIMASK,r0;\n\t"               \
    ::[input] "m" (cpuSR) : "r0");      \
  } while(0)

#define DEFINE_CRITICAL() CpuCriticalVar()
#define ENTER_CRITICAL()  CpuEnterCritical()
#define EXIT_CRITICAL()   CpuExitCritical()


// Flag used in interrupt
volatile bool TCC0_OVF_FLAG = false;
volatile bool TCC0_OVF_FLAG_DEFAULT = false;
volatile uint32_t PULSE_CTR = 0;
volatile bool EMERGENCY_STOP = false;

// Needed for stopping from interrupt
volatile uint32_t loop_len;
volatile uint32_t rampup_len;
volatile int32_t rampdown_cmp;


/*	For DEBUGING with an oscilloscope
 *
 * 	**WARNING the max frequency is about 300 kHz when this code is defined because it
 *    adds a small delay. In normal operation the frequency max is 580 kHz.
 *
 * 	- Toggles pin A1 to see time spent to update the register REG_TCC0_PERB
 * 	- Toggles pin A2 to see time spent in TCC0_Handler
 */
//#define DEBUG_WAVEFORM_GENERATION
#if defined(DEBUG_WAVEFORM_GENERATION)
uint32_t pin0;
volatile uint32_t *mode0;
volatile uint32_t *out0;

uint32_t pin1;
volatile uint32_t *mode1;
volatile uint32_t *out1;
#endif




WaveformGeneration::WaveformGeneration(PwmPin pwm_pin, WaveformGenerationErr* p_err)
: pwm_pin(pwm_pin),
  REG_COMPARE_CHANNEL_X(this->get_ccx(p_err))
{
	if (err_get(p_err)) {return;}

	map_pins(p_err);
	if (err_get(p_err)) {return;}
}
WaveformGeneration::~WaveformGeneration()
{
	if (this->waveform_on) {
		stop_dual_slope_pwm();
	}

}


RwReg& WaveformGeneration::get_ccx(WaveformGenerationErr* p_err) {
	switch(this->pwm_pin) {

		case PwmPin::D0:
			return REG_TCC0_CCB0;

		case PwmPin::D1:
			return REG_TCC0_CCB1;

		case PwmPin::D2:
			return REG_TCC0_CCB2;

		case PwmPin::D3:
			return REG_TCC0_CCB3;

		case PwmPin::D4:
			return REG_TCC0_CCB0;

		case PwmPin::D5:
			return REG_TCC0_CCB1;

		case PwmPin::D6:
			return REG_TCC0_CCB2;

		case PwmPin::D7:
			return REG_TCC0_CCB3;

		case PwmPin::A3:
			return REG_TCC0_CCB0;

		case PwmPin::A4:
			return REG_TCC0_CCB1;

		default:
			*p_err = WaveformGenerationErr::INVALID_PWM_PIN;
			return REG_TCC0_CCB0;
	}

}


void WaveformGeneration::map_pins(WaveformGenerationErr* p_err)
{
	switch(this->pwm_pin) {
		case PwmPin::D0:
			PORT->Group[g_APinDescription[0].ulPort].PINCFG[g_APinDescription[0].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[1].ulPort].PMUX[g_APinDescription[1].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
			break;

		case PwmPin::D1:
			PORT->Group[g_APinDescription[1].ulPort].PINCFG[g_APinDescription[1].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[0].ulPort].PMUX[g_APinDescription[0].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
			break;

		case PwmPin::D2:
			PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[3].ulPort].PMUX[g_APinDescription[3].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
			break;

		case PwmPin::D3:
			PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
			break;

		case PwmPin::D4:
			PORT->Group[g_APinDescription[4].ulPort].PINCFG[g_APinDescription[4].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[5].ulPort].PMUX[g_APinDescription[5].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
			break;

		case PwmPin::D5:
			PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
			break;

		case PwmPin::D6:
			PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[7].ulPort].PMUX[g_APinDescription[7].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
			break;

		case PwmPin::D7:
			PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F;
			break;

		case PwmPin::A3:
			PORT->Group[g_APinDescription[18].ulPort].PINCFG[g_APinDescription[18].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[19].ulPort].PMUX[g_APinDescription[19].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;
			break;

		case PwmPin::A4:
			PORT->Group[g_APinDescription[19].ulPort].PINCFG[g_APinDescription[19].ulPin].bit.PMUXEN = 1;
			// Connect the TCC0 timer to digital output D3 - port pins are paired odd PMUO and even PMUXE
			// F & E specify the timers: TCC0, TCC1 and TCC2
			PORT->Group[g_APinDescription[18].ulPort].PMUX[g_APinDescription[18].ulPin >> 1].reg = PORT_PMUX_PMUXO_E | PORT_PMUX_PMUXE_E;
			break;

		default:
			*p_err = WaveformGenerationErr::PIN_MAPPING_FAILED;
			return;
	}
}


uint32_t WaveformGeneration::calculate_top_dual_slope_pwm(uint32_t freq, WaveformGenerationErr* p_err) {

	if (freq == 0) {
		*p_err = WaveformGenerationErr::INVALID_FREQUENCY;
		return 0;
	}
	else if (freq > MAX_FREQUENCY) {
		*p_err = WaveformGenerationErr::MAX_FREQUENCY;
		return 0;
	}

	uint32_t top = F_GCLK4_TCC0 / (2* freq);	// TODO add prescaling N factor

	if (top == 0) {
		*p_err = WaveformGenerationErr::INVALID_TOP;
		return 0;
	}
	else if (top > 16777215) {		// 24 bit counter
		*p_err = WaveformGenerationErr::MAX_TOP;
		return 0;
	}

	return top;
}



void WaveformGeneration::start_dual_slope_pwm(uint32_t top, uint16_t ccx, bool disable_systick, bool disable_usbserial)
{



	// Stop interrupts
	noInterrupts();
	//DEFINE_CRITICAL();
	//ENTER_CRITICAL();

#if defined(DEBUG_WAVEFORM_GENERATION)
	// Set pin A1 and A2 to output
	pin0 = digitalPinToBitMask(A1);
	mode0 = portModeRegister(digitalPinToPort(A1));
	out0 = portOutputRegister(digitalPinToPort(A1));

	pin1 = digitalPinToBitMask(A2);
	mode1 = portModeRegister(digitalPinToPort(A2));
	out1 = portOutputRegister(digitalPinToPort(A2));

	*mode0 |= pin0;
	*mode1 |= pin1;
#endif


	if (disable_systick) {
		// Disable systick timer (millis(), delay())
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	}

	if (disable_usbserial) {
		// Disable USB
		// https://github.com/arduino/ArduinoCore-samd/issues/103
		USB->DEVICE.CTRLB.bit.DETACH = 1;
		USB->DEVICE.INTFLAG.bit.SUSPEND = 1;
		while (USB->DEVICE.SYNCBUSY.bit.ENABLE == 1);
	}

	// Set up the generic clock (GCLK4) used to clock timers
	// https://forum.arduino.cc/index.php?topic=396201.0
	// https://forum.arduino.cc/index.php?topic=346731.0
	REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 3: 48MHz/3=16MHz
	                  GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

	REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
	                   GCLK_GENCTRL_GENEN |         // Enable GCLK4
	                   GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
	                   GCLK_GENCTRL_ID(4);          // Select GCLK4
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


	// Feed GCLK4 to TCC0 and TCC1
	REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
	                   GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
	                   GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
	while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


	// Interrupts
	TCC0->INTENSET.reg = 0;                 		// Disable all interrupts
	TCC0->INTENSET.bit.OVF = 1;          			// Enable overfollow
	TCC0->INTENSET.bit.MC0 = 0;

	// Enable InterruptVector
	NVIC_ClearPendingIRQ(TCC0_IRQn);                // Clear pending for timer interrupt
	NVIC_SetPriority(TCC0_IRQn, 0);                 // Set interrupt to highest priority
	NVIC_EnableIRQ(TCC0_IRQn);                      // Enable the interrupt event handler



	// Read counter
	//while (TCC0->SYNCBUSY.bit.CTRLB);
	//TCC0->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val;
	//while (TCC0->SYNCBUSY.bit.CTRLB);
	//while (TCC0->SYNCBUSY.bit.COUNT);


	// Set TCC0 count to 0
	REG_TCC0_COUNT = 0x000000;
	while (TCC0->SYNCBUSY.bit.COUNT);


	// Dual slope PWM operation: timers countinuously count up to PER register value then down 0
	REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         		// Reverse the output polarity on all TCC0 outputs
	                 TCC_WAVE_WAVEGEN_DSBOTTOM;    		// Setup dual slope PWM on TCC0

	while (TCC0->SYNCBUSY.bit.WAVE);               		// Wait for synchronization

	// Each timer counts up to a maximum or TOP value set by the PER register,
	// this determines the frequency of the PWM operation:

	// ** PER is used only for initialisation. Use PERB elsewhere
	REG_TCC0_PER = top;         						// Set the frequency of the PWM on TCC0 to 250kHz
	while (TCC0->SYNCBUSY.bit.PER);                		// Wait for synchronization

	REG_COMPARE_CHANNEL_X = ccx;

	while (TCC0->SYNCBUSY.bit.CCB0);                	// Wait for synchronization
	while (TCC0->SYNCBUSY.bit.CCB1);                	// Wait for synchronization
	while (TCC0->SYNCBUSY.bit.CCB2);                	// Wait for synchronization
	while (TCC0->SYNCBUSY.bit.CCB3);                	// Wait for synchronization


	// Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
	REG_TCC0_CTRLA |= TCC_CTRLA_PRESCSYNC_PRESC |
			  	  	  TCC_CTRLA_PRESCALER_DIV1 |    	// Divide GCLK4 by 1
	                  TCC_CTRLA_ENABLE;             	// Enable the TCC0 output
	while (TCC0->SYNCBUSY.bit.ENABLE);              	// Wait for synchronization

	// Enable interrupts
	//EXIT_CRITICAL();
	interrupts();

}


void WaveformGeneration::stop_generation() {
	TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	while (TCC0->SYNCBUSY.bit.ENABLE); // wait for sync
	//TCC0->INTENSET.reg = 0;                 		// Disable all interrupts
	NVIC_DisableIRQ(TCC0_IRQn);                // Clear pending for timer interrupt
	NVIC_ClearPendingIRQ(TCC0_IRQn);

	loop_len = 0;				// Up counter
	rampup_len = 0;				// Up counter
	rampdown_cmp = 0x7FFFFFFF;	// Down counter

	TCC0_OVF_FLAG_DEFAULT = true;	// To break the loop
	TCC0_OVF_FLAG = true;			// To break the loop
	EMERGENCY_STOP = true;
}



void WaveformGeneration::stop_dual_slope_pwm(bool  disable_systick, bool  disable_usbserial)
{
	// Stop interrupts
	//DEFINE_CRITICAL();
	//ENTER_CRITICAL();
	noInterrupts();

	// Disable TCC0
	TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	while (TCC0->SYNCBUSY.bit.ENABLE); // wait for sync

	// Reset TCC0
	TCC0->CTRLA.bit.SWRST = 1;
	while (TCC0->SYNCBUSY.bit.SWRST); // wait for sync

	TCC0->INTENSET.reg = 0;                 		// Disable all interrupts
	NVIC_DisableIRQ(TCC0_IRQn);                // Clear pending for timer interrupt
	NVIC_ClearPendingIRQ(TCC0_IRQn);


	// Set these values in case this function was triggered by an interrupt
	loop_len = 0;				// Up counter
	rampup_len = 0;				// Up counter
	rampdown_cmp = 0x7FFFFFFF;	// Down counter

	TCC0_OVF_FLAG_DEFAULT = true;	// To break the loop
	TCC0_OVF_FLAG = true;			// To break the loop


	// Restore configuration for systick
	if (disable_systick) {
		SysTick->CTRL  |= SysTick_CTRL_ENABLE_Msk;
	}

	if (disable_usbserial) {
		// Restore configuration for usb serial
		USB->DEVICE.CTRLA.bit.ENABLE = 1;
		while (USB->DEVICE.SYNCBUSY.bit.ENABLE);
		USB->DEVICE.CTRLB.bit.DETACH = 0;
	}

	// Enable interrupts
	//EXIT_CRITICAL();
	interrupts();
}


void WaveformGeneration::gen_dual_slope_pwm(bm_compact bm, uint32_t buf[], WaveformGenerationErr* p_err, bool disable_systick, bool disable_usbserial)
{
	if (!bm.is_converted) {
		*p_err = WaveformGenerationErr::FREQ_CONVERSION_NOT_DONE;
		return;
	}


	uint32_t rampup_ctr;	// Initialized in switch case
	uint32_t loop_ctr;		// Initialized in switch case
	int32_t rampdown_ctr = bm.rampdown_idx;

	uint32_t loop_freq = bm.plateau_freq;

	// Variables that will be set to
	loop_len = bm.plateau_len;
	rampup_len = bm.rampup_len;
	rampdown_cmp = 0;



	PULSE_CTR = 0;
	this->waveform_on = true;
	TCC0_OVF_FLAG = false;
	TCC0_OVF_FLAG_DEFAULT = false;
	EMERGENCY_STOP = false;

	switch (bm.mode) {

		case BmCompactType::SINGLE:
			start_dual_slope_pwm(loop_freq+(CCX/2), CCX, disable_systick, disable_usbserial);
			while(!TCC0_OVF_FLAG && !EMERGENCY_STOP);
			REG_COMPARE_CHANNEL_X  = 0;
			stop_dual_slope_pwm(disable_systick, disable_usbserial);
			break;

		case BmCompactType::CONSTANT:
			loop_ctr = 1;
			start_dual_slope_pwm(loop_freq+(CCX/2), CCX, disable_systick, disable_usbserial);
			for(; loop_ctr < loop_len; loop_ctr++)
			{
				while(!TCC0_OVF_FLAG);
				//*out0 ^= pin0;
				REG_TCC0_PERB = loop_freq;
				TCC0_OVF_FLAG = TCC0_OVF_FLAG_DEFAULT;
				//*out0 ^= pin0;
			}
			//Serial.println("c4");
			//Serial.print("TCC0_OVF_FLAG = ");
			//Serial.println(TCC0_OVF_FLAG);
			while(!TCC0_OVF_FLAG && !EMERGENCY_STOP);
			//Serial.println("d4");
			REG_COMPARE_CHANNEL_X  = 0;
			stop_dual_slope_pwm(disable_systick, disable_usbserial);
			break;

		case BmCompactType::RAMPUP_RAMPDOWN:
			rampup_ctr = 1;
			start_dual_slope_pwm(buf[0]+(CCX/2), CCX, disable_systick, disable_usbserial);
			// Rampup
			for(; rampup_ctr < rampup_len; rampup_ctr++)
			{
				while(!TCC0_OVF_FLAG);
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
				REG_TCC0_PERB = buf[rampup_ctr];
				TCC0_OVF_FLAG = TCC0_OVF_FLAG_DEFAULT;
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
			}
			// Rampdown
			for(; rampdown_ctr >= rampdown_cmp; rampdown_ctr--)
			{
				while(!TCC0_OVF_FLAG);
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
				REG_TCC0_PERB = buf[rampdown_ctr];
				TCC0_OVF_FLAG = TCC0_OVF_FLAG_DEFAULT;
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
			}
			while(!TCC0_OVF_FLAG && !EMERGENCY_STOP);
			REG_COMPARE_CHANNEL_X  = 0;
			stop_dual_slope_pwm(disable_systick, disable_usbserial);
			break;

		case BmCompactType::FULL:
			rampup_ctr = 1;
			loop_ctr = 0;
			start_dual_slope_pwm(buf[0]+(CCX/2), CCX, disable_systick, disable_usbserial);
			// Rampup
			for(; rampup_ctr < rampup_len; rampup_ctr++)
			{
				while(!TCC0_OVF_FLAG);
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
				REG_TCC0_PERB = buf[rampup_ctr];
				TCC0_OVF_FLAG = TCC0_OVF_FLAG_DEFAULT;
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
			}
			// Plateau
			for(; loop_ctr < loop_len; loop_ctr++)
			{
				while(!TCC0_OVF_FLAG);
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
				REG_TCC0_PERB = loop_freq;
				TCC0_OVF_FLAG = TCC0_OVF_FLAG_DEFAULT;
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
			}
			// Rampdown
			for(; rampdown_ctr >= rampdown_cmp; rampdown_ctr--)
			{
				while(!TCC0_OVF_FLAG);
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
				REG_TCC0_PERB = buf[rampdown_ctr];
				TCC0_OVF_FLAG = TCC0_OVF_FLAG_DEFAULT;
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out0 ^= pin0;
#endif
			}
			while(!TCC0_OVF_FLAG && !EMERGENCY_STOP);
			REG_COMPARE_CHANNEL_X  = 0;
			stop_dual_slope_pwm(disable_systick, disable_usbserial);
			break;
	}
	TCC0_OVF_FLAG = false;
	TCC0_OVF_FLAG_DEFAULT = false;
	EMERGENCY_STOP = false;
	this->waveform_on = false;

	if (PULSE_CTR != bm.steps) {
		//delay(5000);
		Serial.print("PULSE_CTR = ");
		Serial.println(PULSE_CTR);
		Serial.print(" bm.steps = ");
		Serial.println( bm.steps);

		*p_err = WaveformGenerationErr::PULSE_MISMATCH;
	}
}

// https://www.avrfreaks.net/forum/tcc0-ppw-capture
void TCC0_Handler() {
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out1 ^= pin1;
#endif
	TCC0_OVF_FLAG = true;
	TCC0->INTFLAG.bit.OVF = 1;    // Clear OVF flag

	PULSE_CTR++;
#if defined(DEBUG_WAVEFORM_GENERATION)
				*out1 ^= pin1;
#endif
}




bool WaveformGeneration::err_get(WaveformGenerationErr* p_err)
{
	if (*p_err == WaveformGenerationErr::NONE) {
		return false;
	}
	return true;
}

void WaveformGeneration::err_clear(WaveformGenerationErr* p_err)
{
	*p_err = WaveformGenerationErr::NONE;
}


void WaveformGeneration::convert_frequencies(bm_compact* bm, uint32_t buf[], WaveformGenerationErr* p_err)
{

	if (bm->is_converted) {
		*p_err = WaveformGenerationErr::FREQ_CONVERSION_ALREADY_DONE;
		return;
	}


	uint32_t period;
	// Convert frequency to period in buffer
	for(uint32_t i = 0; i < bm->buf_size; i++){
		period = calculate_top_dual_slope_pwm(buf[i], p_err);
		if (err_get(p_err)) {goto error;}
		buf[i] = period;
	}

	// Convert frequency to period for plateau_freq
	if (bm->mode != BmCompactType::RAMPUP_RAMPDOWN) {
		period = calculate_top_dual_slope_pwm(bm->plateau_freq, p_err);
		if (err_get(p_err)) {goto error;}
		bm->plateau_freq = period;
	}
	bm->is_converted = true;
	return;

error:
	bm->is_converted = true;
}



#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO

#endif // ARDUINO
