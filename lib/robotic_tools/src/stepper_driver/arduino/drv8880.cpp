/*
 * drv8880.cpp
 *
 *  Created on: May 23, 2018
 *      Author: robotic_tools
 */


#include <robotic_tools/stepper_driver/arduino/drv8880.h>
#include <robotic_tools/error_handle/error_handle.h>

uint16_t DRV8880::getMicrostep(MicrostepMode mres_mode)
{
    switch(mres_mode){

    	case MicrostepMode::FULL_STEP:
            return 1;
            break;
        case MicrostepMode::HALF_STEP:
        	return 2;
            break;
        case MicrostepMode::HALF_STEP_CIRCULAR:
        	return 2;
            break;
        case MicrostepMode::QUARTER_STEP:
        	return 4;
            break;
        case MicrostepMode::EIGHTH_STEP:
        	return 8;
            break;
        case MicrostepMode::SIXTEENTH_STEP:
            return 16;
            break;
        default:
        	p_err->set(DRV8880ErrCode::MICROSTEP_INVALID_PARAMETER);
        	return 0;
    }
}









#if defined(ARDUINO)




#include <Arduino.h>


/*
 * Basic connection: only DIR, STEP are connected.
 * Microstepping controls should be hardwired.
 */



/*
DRV8880::DRV8880(short steps, short dir_pin, short step_pin)
:BasicStepperDriver(steps, step_pin, dir_pin)
{
	setDirection(this->direction);
}


DRV8880::DRV8880(short steps, short dir_pin, short step_pin, short enable_pin)
:BasicStepperDriver(steps, dir_pin, step_pin, enable_pin)
{
	setDirection(this->direction);
}
*/
/*
 * Fully wired. All the necessary control pins for DRV8880 are connected.
 */
DRV8880::DRV8880(short steps, short dir_pin, Trajectory* trajectory, short m0, short m1)
:BasicStepperDriver(steps, dir_pin, trajectory), m0(m0), m1(m1)
{
	setDirection(this->direction);
	setMicrostepMode(this->microstep_mode);
}

DRV8880::DRV8880(short steps, short dir_pin, short enable_pin, Trajectory* trajectory, short m0, short m1)
:BasicStepperDriver(steps, dir_pin, enable_pin, trajectory), m0(m0), m1(m1)
{
	setDirection(this->direction);
	setMicrostepMode(this->microstep_mode);
}

DRV8880::DRV8880(short steps, short dir_pin, Trajectory* trajectory, short m0, short m1, short trq0, short trq1)
:BasicStepperDriver(steps, dir_pin, trajectory), m0(m0), m1(m1), trq0(trq0), trq1(trq1)
{
	setDirection(this->direction);
	setMicrostepMode(this->microstep_mode);
	setCurrent(this->current);
}

DRV8880::DRV8880(short steps, short dir_pin, short enable_pin, Trajectory* trajectory, short m0, short m1, short trq0, short trq1)
:BasicStepperDriver(steps, dir_pin, enable_pin, trajectory), m0(m0), m1(m1), trq0(trq0), trq1(trq1)
{
	setDirection(this->direction);
	setMicrostepMode(this->microstep_mode);
	setCurrent(this->current);
}






/*
 * Set microstepping mode (1:divisor)
 * Allowed ranges for DRV8880 are 1:1 to 1:16
 * If the control pins are not connected, we recalculate the timing only
 */
void DRV8880::setMicrostepMode(MicrostepMode microstep_mode){

    if (!IS_CONNECTED(m0) || !IS_CONNECTED(m1)){
    	p_err->set(DRV8880ErrCode::MICROSTEP_PINS_NOT_SET);
    	return;
    }


    /*
     * Step mode truth table
     * M1 M0    step mode
     *  0  0     1
     *  1  0     2
     *  1  1     4
     *  0  Z     8
     *  1  Z    16
     *
     *  0  1     3 (non-circular)
     *  Z = high impedance mode (M0 is tri-state)
     */




    switch(microstep_mode){

    	case MicrostepMode::FULL_STEP:
    	    pinMode(m1, OUTPUT);
    	    pinMode(m0, OUTPUT);
            digitalWrite(m1, LOW);
            digitalWrite(m0, LOW);
            this->microstep = 1;
            break;
        case MicrostepMode::HALF_STEP:
            pinMode(m1, OUTPUT);
            pinMode(m0, OUTPUT);
            digitalWrite(m1, HIGH);
            digitalWrite(m0, LOW);
            this->microstep = 2;
            break;
        case MicrostepMode::HALF_STEP_CIRCULAR:
            pinMode(m1, OUTPUT);
            pinMode(m0, OUTPUT);
            digitalWrite(m1, LOW);
            digitalWrite(m0, HIGH);
            this->microstep = 2;
            break;
        case MicrostepMode::QUARTER_STEP:
            pinMode(m1, OUTPUT);
            pinMode(m0, OUTPUT);
            digitalWrite(m1, HIGH);
            digitalWrite(m0, HIGH);
            this->microstep = 4;
            break;
        case MicrostepMode::EIGHTH_STEP:
            pinMode(m1, OUTPUT);
            digitalWrite(m1, LOW);
            pinMode(m0, INPUT); // Z - high impedance
            this->microstep = 8;
            break;
        case MicrostepMode::SIXTEENTH_STEP:
            pinMode(m1, OUTPUT);
            digitalWrite(m1, HIGH);
            pinMode(m0, INPUT); // Z - high impedance
            this->microstep = 16;
            break;
        default:
        	p_err->set(DRV8880ErrCode::MICROSTEP_INVALID_PARAMETER);
        	return;
    }

    this->microstep_mode = microstep_mode;

}

DRV8880::MicrostepMode DRV8880::getMicrostepMode(){

	return microstep_mode;
}

uint16_t DRV8880::getMicrostep()
{
	return microstep;
}






DRV8880::Current DRV8880::getCurrent(){

	return current;
}

void DRV8880::setCurrent(Current current){
    /*
     * Torque DAC Settings table
     * TRQ1 TRQ0 Current scalar
     *  1     1       25%
     *  1     0       50%
     *  0     1       75%
     *  0     0      100%
     */
    if (!IS_CONNECTED(trq1) || !IS_CONNECTED(trq0)){
    	p_err->set(DRV8880ErrCode::CURRENT_PINS_NOT_SET);
    	return;
    }

    pinMode(trq1, OUTPUT);
    pinMode(trq0, OUTPUT);


    switch(current){
        case Current::PERCENT_25:
            digitalWrite(trq0, HIGH);
            digitalWrite(trq1, HIGH);
            break;

        case Current::PERCENT_50:
            digitalWrite(trq0, LOW);
            digitalWrite(trq1, HIGH);
            break;

        case Current::PERCENT_75:
            digitalWrite(trq0, HIGH);
            digitalWrite(trq1, LOW);
            break;

        case Current::PERCENT_100:
            digitalWrite(trq0, LOW);
            digitalWrite(trq1, LOW);
        	break;

        default:
        	p_err->set(DRV8880ErrCode::CURRENT_INVALID_PARAMETER);
        	return;
    }

    this->current = current;
}


void DRV8880::setDirection(Direction dir) {
	digitalWrite(dir_pin, (unsigned char)dir);
	this->direction = dir;
}

DRV8880::Direction DRV8880::getDirection() {
	return direction;
}





#endif








