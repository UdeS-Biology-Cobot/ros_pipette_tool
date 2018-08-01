/*
 * basic_stepper_driver.cpp
 *
 *  Created on: May 22, 2018
 *      Author: robotic_tools
 */

#if defined(ARDUINO)


#include <robotic_tools/stepper_driver/arduino/basic_stepper_driver.h>

//#include "basic_stepper_driver.h"
#include <Arduino.h>
//#include <error_handle.h>


/*
 * Basic connection: only DIR, STEP are connected.
 * Microstepping controls should be hardwired.
 */



/*
BasicStepperDriver::BasicStepperDriver(short steps, short dir_pin, short step_pin)
:motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin), trajectory(nullptr)
{
	begin();
}

BasicStepperDriver::BasicStepperDriver(short steps, short dir_pin, short step_pin, short enable_pin)
:motor_steps(steps), dir_pin(dir_pin), step_pin(step_pin), enable_pin(enable_pin), trajectory(nullptr)
{
	begin();
}
*/

BasicStepperDriver::BasicStepperDriver(short steps, short dir_pin, Trajectory* trajectory)
:motor_steps(steps), dir_pin(dir_pin), trajectory(trajectory)
{
	step_pin = trajectory->wave_gen->get_waveform_pin();
	begin();
}



BasicStepperDriver::BasicStepperDriver(short steps, short dir_pin, short enable_pin, Trajectory* trajectory)
:motor_steps(steps), dir_pin(dir_pin), enable_pin(enable_pin), trajectory(trajectory)
{
	step_pin = trajectory->wave_gen->get_waveform_pin();
	begin();
}







void BasicStepperDriver::begin() {
	pinMode(dir_pin, OUTPUT);
	pinMode(step_pin, OUTPUT);

    if IS_CONNECTED(enable_pin){
        pinMode(enable_pin, OUTPUT);
        digitalWrite(enable_pin, HIGH); // disable
    }
}


void BasicStepperDriver::generate_trajectory(Trajectory::Profile profile, uint32_t params[])
{
	trajectory->generate(profile, params);
}

void BasicStepperDriver::move()
{
	trajectory->run();
}

#endif
