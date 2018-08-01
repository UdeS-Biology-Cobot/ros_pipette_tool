/*
 * basic_stepper_driver.h
 *
 *  Created on: May 22, 2018
 *      Author: robotic_tools
 */

#ifndef BASIC_STEPPER_DRIVER_H_
#define BASIC_STEPPER_DRIVER_H_

#include <robotic_tools/trajectory/trajectory.h>

//#include <error_handle.h>



#define PIN_UNCONNECTED -1
#define IS_CONNECTED(pin) (pin != PIN_UNCONNECTED)

#define PIN_UNCONNECTED -1


class BasicStepperDriver {

protected:
	/*
     * Motor Configuration
     */
	short motor_steps;           	/* motor steps per revolution (usually 200)	*/

    /*
     * Driver Configuration
     */
    short dir_pin;
    short step_pin;
    short enable_pin;


    //short microsteps = 1;

    //virtual short getMaxMicrostep();


    //~BasicStepperDriver();

private:
    // microstep range (1, 16, 32 etc)
    //static const short MAX_MICROSTEP = 128;




public:
    /* Control over */
    //BasicStepperDriver(short steps, short dir_pin, short step_pin);
    //BasicStepperDriver(short steps, short dir_pin, short step_pin, short enable_pin);



    BasicStepperDriver(short steps, short dir_pin, Trajectory*);
    BasicStepperDriver(short steps, short dir_pin, short enable_pin, Trajectory*);



    void generate_trajectory(Trajectory::Profile profile, uint32_t params[]);
    void move();

    /* TODO add function that checks that all pins are not the same */
    Trajectory* trajectory;


private:
    void begin();
};



#endif /* BASIC_STEPPER_DRIVER_H_ */
