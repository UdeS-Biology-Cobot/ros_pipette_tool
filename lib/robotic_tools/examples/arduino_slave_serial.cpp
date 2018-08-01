

//#define TRAJECTORY_BUFFER_SIZE 256


//#include <boost/static_assert.hpp>
#include "Arduino.h"


#include <biobot_arduino.h>





ErrorHandle err_handle = ErrorHandle();
ErrorHandle* p_err = &err_handle;



#define BAUD_RATE 	 1000000uL
#define BAUD_CONFIG  SERIAL_8N1

#define MOTOR_STEPS 400
#define MOTOR_VEL_MAX 16
#define MOTOR_ACC_MAX 0

float LEADCREW_TRAVEL_PER_TURN = 2.384976525821596;


#define DIR_PIN 4
#define STEP_PIN 10
#define M0_PIN 	7
#define M1_PIN 	8

#define RS485_CONTROL_PIN 3


#define SLAVE_ADDRESS 0x01

#define SERIAL_NUMBER 0x7E



void setup ()
{
	//Serial.begin(BAUD_RATE, SERIAL_8N1);
}






// The loop function is called in an endless loop
void loop()
{

#ifdef MACRO
	PipetteToolProtocolConfig conf;

	conf.motor_step 					= 400;
	conf.motor_vel_max 					= 16;
	conf.motor_acc_max					= 0;

	conf.leadscrew_thread_dir			= Leadscrew::ThreadDir::RIGHT_HAND;
	conf.leadscrew_travel_per_turn_fm	= 2384976525821596;



	DRV8880 drv8880 = DRV8880(MOTOR_STEPS, DIR_PIN, nullptr, M0_PIN, M1_PIN);
	DRV8880Register drv8880_reg = DRV8880Register(&drv8880);

	StepperDriverRegister driver_reg(&drv8880_reg);
	conf.driver = &driver_reg;


	PwmPfc pwm_pfc = PwmPfc();
	WaveformGeneration wave_gen(&pwm_pfc);
	conf.waveform_generation = &wave_gen;
#endif
/*
	WaveformGeneration* waveform_generation;


	StepperDriver* driver;



	PipetteTool::Type pipette_type;
	uint32_t pipette_serial;

*/


	StepperMotor motor = StepperMotor(MOTOR_STEPS, MOTOR_VEL_MAX, MOTOR_ACC_MAX);


	Leadscrew leadscrew(Leadscrew::ThreadDir::RIGHT_HAND, LEADCREW_TRAVEL_PER_TURN);

	// PWM Phase and Frequency Correct waveform mode
	PwmPfc pwm_pfc = PwmPfc();

	// Abstract class for all waveform generation modes
	WaveformGeneration wave_gen(&pwm_pfc);

	// Profile and waveform generation
	Trajectory trajectory(&wave_gen, F_CPU);

	// Stepper driver with Trajectory planning(step pin) and microstep configuration
	DRV8880 drv8880 = DRV8880(MOTOR_STEPS, DIR_PIN, &trajectory, M0_PIN, M1_PIN);

	StepperDriver driver(&drv8880);


	// Slave protocol design for DRV8880 and Trajectory planning
	PipetteTool pipette_tool(&motor, &driver, &leadscrew, &trajectory, PipetteTool::Type::GILSON_P200, SERIAL_NUMBER);



	PipetteToolSlaveRegister pipette_reg = PipetteToolSlaveRegister(&pipette_tool, StepperDriver::Type::DRV8880);
	StepperMotorSlaveRegister motor_reg = StepperMotorSlaveRegister(&motor);
	DRV8880SlaveRegister drv8880_reg = DRV8880SlaveRegister(&drv8880);
	StepperDriverRegister driver_reg = StepperDriverRegister(&drv8880_reg);
	TrajectorySlaveRegister trajectory_reg(&trajectory);
	LeadscrewSlaveRegister leadscrew_reg = LeadscrewSlaveRegister(&leadscrew);


	PipetteToolSlaveProtocol pipette_prot(&pipette_reg, &motor_reg, &driver_reg, &trajectory_reg, &leadscrew_reg, SLAVE_ADDRESS);





	// Serial communication with slave protocol
	PipetteToolSerialSlave pipette_serial(&pipette_prot, &Serial1, BAUD_RATE);
	//pipette_serial.start();

	//drv8880.setMicrostepMode(DRV8880::MicrostepMode::FULL_STEP);
	//drv8880_reg.set_register(DRV8880SlaveRegister::Registers::CONFIG, 0x00000000);

	pipette_serial.start();
	/*

*/

/*
	WaveformGeneration wave_gen(WaveformGeneration::Mode::PWM_PHASE_FREQUENCY_CORRECT);

	Trajectory trajectory(&wave_gen, buf, BUFFER_SIZE);


	uint32_t params[16];

	uint32_t steps = 400;
	uint32_t f_min = 500;
	uint32_t f_max = 9600;
	uint32_t slope = 50;

	params[0] = steps;
	params[1] = f_min;
	params[2] = f_max;
	params[3] = slope;

	DRV8880 stepper(MOTOR_STEPS, DIR_PIN, &trajectory, M0_PIN, M1_PIN);

	stepper.generate_trajectory(Trajectory::Profile::CONCAVE_SCURVE_VEL, params);
	stepper.move();

	delay(10000);
*/
}
