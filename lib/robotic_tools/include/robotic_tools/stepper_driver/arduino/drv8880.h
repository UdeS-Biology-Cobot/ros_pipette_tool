/*
 * drv8880.h
 *
 *  Created on: May 23, 2018
 *      Author: robotic_tools
 */

#ifndef DRV8880_H_
#define DRV8880_H_

#include <robotic_tools/stepper_driver/arduino/basic_stepper_driver.h>






class DRV8880 : public BasicStepperDriver {
protected:
    short m0 = PIN_UNCONNECTED;
    short m1 = PIN_UNCONNECTED;
    short trq0 = PIN_UNCONNECTED;
    short trq1 = PIN_UNCONNECTED;


public:

    /*
     * Basic connection: only DIR, STEP are connected.
     * Microstepping controls should be hardwired.
     */
    //DRV8880(short steps, short dir_pin, short step_pin);
    //DRV8880(short steps, short dir_pin, short step_pin, short enable_pin);
    /*
     * DIR, STEP and microstep control M0, M1
     */
    DRV8880(short steps, short dir_pin, Trajectory*, short m0, short m1);
    DRV8880(short steps, short dir_pin, short enable_pin, Trajectory*, short m0, short m1);
    /*
     * Fully Wired - DIR, STEP, microstep and current control
     */
    DRV8880(short steps, short dir_pin, Trajectory*, short m0, short m1, short trq0, short trq1);
    DRV8880(short steps, short dir_pin, short enable_pin, Trajectory*, short m0, short m1, short trq0, short trq1);



	enum class MicrostepMode : unsigned char
	{
		FULL_STEP,
		HALF_STEP,
		HALF_STEP_CIRCULAR,
		QUARTER_STEP,
		EIGHTH_STEP,
		SIXTEENTH_STEP,
	};

	enum class Current : unsigned char
	{
		PERCENT_25 = 25,
		PERCENT_50 = 50,
		PERCENT_75 = 75,
		PERCENT_100 = 100,
	};

	enum class Direction : bool
	{
		CLOCKWISE = 0,			/* Stepper Channel A = Sine, B = Cosine */
		COUNTER_CLOCKWISE = 1,
	};

	MicrostepMode microstep_mode = MicrostepMode::FULL_STEP;
	uint16_t microstep = getMicrostep(microstep_mode);
	Current current = Current::PERCENT_25;
	Direction direction = Direction::CLOCKWISE;


    void setMicrostepMode(MicrostepMode microstep);
    void setCurrent(Current percent);
    void setDirection(Direction dir);

    uint16_t getMicrostep();
    static uint16_t getMicrostep(MicrostepMode);

    MicrostepMode getMicrostepMode();


    Current getCurrent();
    Direction getDirection();






};



#endif /* DRV8880_H_ */
