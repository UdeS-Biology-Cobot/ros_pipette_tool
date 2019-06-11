/*
 * pipette_tool.h
 *
 *  Created on: May 24, 2018
 *      Author: biobot
 */

#ifndef PIPETTE_TOOL_H_
#define PIPETTE_TOOL_H_



#if defined(ARDUINO)
#if defined(ARDUINO_SAMD_MKRZERO)

#include <robotic_tool/pipette_tool/arduino/pipette_tool_storage.h>
#include <robotic_tool/sensor/drv5023.h>

#include <robotic_tool/pipette_tool/interface/driver_interface.h>
#include <robotic_tool/pipette_tool/interface/encoder_interface.h>

#include <robotic_tool/stepper_motor/stepper_motor.h>
#include <robotic_tool/leadscrew/leadscrew.h>

#include <robotic_tool/trajectory/motion_profile/motion_profile.h>
#include <robotic_tool/trajectory/arduino/waveform_generation.h>


//#include <biobot/stepper_motor/stepper_motor.h>
//#include <biobot/leadscrew/leadscrew.h>
//#include <biobot/trajectory/trajectory.h>
//#include <biobot/stepper_driver/stepper_driver.h>

enum class PipetteToolErr : uint8_t {
	NONE,
	NOT_INITIALIZED,

	HOMING_ERROR,				// Should save new values in storage

	DRIVER_ERROR,
	MOTION_PROFILE_ERROR,
	WAVEFORM_GENERATION_ERROR,
	HALL_EFFECT_SENSOR_ERROR,
	STORAGE_ERROR,
	ENCODER_ERROR,
	TRAJECTORY_ERROR,
	EMERGENCY_STOP,
	INVALID_SHAFT_MODEL,
	OUT_OF_BOND,
	POSITION_ERROR,
};



struct PtConf {
	uint32_t serial_number;

	uint32_t default_microstep;
	uint32_t homing_microstep;
	uint32_t eject_microstep;

	double homing_speed;
	double eject_speed;

	double max_acc;

	double max_start_speed;
	double default_start_speed;
};


/*
uint32_t EJECT_MICROSTEP = 4;
uint32_t HOMING_MICROSTEP;	// max microstep of driver (initialized in constructor)
uint32_t DEFAULT_MICROSTEP = 16;	// (initialized in constructor)


double EJECT_SPEED 	= 0.03;	// (m/s)
double HOMING_SPEED = 0.0075;	// (m/s)

double MAX_ACC 	= 0.02;		// (m/s^2) 0.02
double MAX_START_SPEED  = 0.008;

double DEFAULT_START_SPEED  = 0.004;
*/

class PipetteTool
{
public:




	enum ShaftModel : uint8_t {
		GILSON_P200,
	};

	enum class DisplacementDirection : bool {
		ASPIRATE,
		DISPENSE
	};

	void init(PipetteToolErr* p_err);

	void aspirate(uint32_t nL, double speed, PipetteToolErr* p_err);
	void dispense(uint32_t nL, double speed, PipetteToolErr* p_err);

	void multiple_sequence(uint32_t nL, double speed, DisplacementDirection start_dir, uint32_t seq_number, PipetteToolErr* p_err);


	void do_homing(PipetteToolErr* p_err);
	void eject_tip(PipetteToolErr* p_err);

	uint32_t get_max_nl(PipetteToolErr* p_err);	/* WARNING max nl changes with number of manipulations. Its up to the application to follow the changes */
	//int32_t get_nl_pos(PipetteToolErr* p_err);	// Get current nanoliter 0 = ENCODER_POS_TIP_SECURE , Negative = ejection zone
	int32_t get_offset_nl();
	double get_max_speed();

	uint32_t get_rem_aspirate_vol_nl(PipetteToolErr* p_err);
	uint32_t get_rem_dispense_vol_nl(PipetteToolErr* p_err);


	uint32_t get_serial_number(PipetteToolErr* p_err);

	void move_to_top(PipetteToolErr* p_err);
	void move_to_tip(PipetteToolErr* p_err);

	// Trajectory related
	void disable_trajectory_systick();
	void disable_trajectory_usbserial();
	void enable_trajectory_systick();
	void enable_trajectory_usbserial();

	bool err_get(PipetteToolErr* p_err);
	void err_clear(PipetteToolErr* p_err);


	PipetteTool(StepperMotor*  motor,
			       Leadscrew*  leadscrew,
			 DriverInterface*  driver,
			EncoderInterface*  encoder,
					 DRV5023*  he_sensor,
			   MotionProfile*  profile,
		  WaveformGeneration*  wf_gen,
		        const PtConf   pt_conf,
		      PipetteToolErr*  p_err);

	// Hardware Components
	StepperMotor* 		motor;
	Leadscrew* 			leadscrew;
	DriverInterface* 	driver;
	EncoderInterface*	encoder;		// Get rotation feedback
	DRV5023* 			he_sensor;		// Hall-effect sensor for initialisation

	// Motion Control
	MotionProfile* 		profile;	// Generate motion profile
	WaveformGeneration*	wf_gen;		// Generate trajectory from motion profile

	// Configuration
	PtConf   pt_conf;

	// Storage
	PipetteToolStorage storage = PipetteToolStorage();

private:

	void homing_sequence(uint32_t trigger, PipetteToolErr* p_err);
	void set_displacement_dir(DisplacementDirection dir, PipetteToolErr* p_err);

	void move(uint32_t nL, double speed, DisplacementDirection dir, PipetteToolErr* p_err);

	void move_to_bottom(PipetteToolErr* p_err);

	void move_to_encoder_pos(int32_t encoder_pos, double speed, uint32_t microstep, PipetteToolErr* p_err);


	int32_t validate_trajectory(uint32_t mstep, int32_t prev_pos, uint32_t microstep, PipetteToolErr* p_err);

	// Conversion functions
	double nl_to_m(int32_t nL, PipetteToolErr* p_err);		// nanoliters to meters
	int32_t m_to_nl(double distance, PipetteToolErr* p_err);
	double nl_to_mm(int32_t nL, PipetteToolErr* p_err);	// nanoliters to millimeters
	int32_t mm_to_nl(double distance, PipetteToolErr* p_err);

	uint32_t nl_to_mstep(uint32_t nL, uint32_t microstep, PipetteToolErr* p_err);	// nanoliters to motor step
	uint32_t m_to_mstep(double distance, uint32_t microstep);						// meters to motor step

	uint32_t mstep_to_encoder_pos(uint32_t mstep, uint32_t microstep);	// motor step to encoder position (depends on resolution)

	uint32_t speed_to_freq(double speed, uint32_t microstep);
	uint32_t acc_to_freq(double vel, double acc, uint32_t microstep);

	uint32_t encoder_pos_to_mstep(int32_t encoder_count ,uint32_t microsteps);
	double encoder_pos_to_distance(int32_t encoder_count);

	double one_mstep_to_distance(uint32_t microsteps);	// Get the distance of one motorstep
	uint32_t distance_to_encoder_pos(double distance);

	bm_compact generate_profile(double speed, const uint32_t steps, uint32_t microstep, PipetteToolErr* p_err);
	void generate_waveform(const bm_compact bm, PipetteToolErr* p_err);
	void generate_trajectory(double speed, const uint32_t steps, uint32_t microstep, PipetteToolErr* p_err);
	void generate_trajectory2(double speed, const uint32_t steps, uint32_t microstep, PipetteToolErr* p_err);

	void calc_encoder_pos_offset();

	void set_volume_formula(ShaftModel _shaft_model, PipetteToolErr* p_err);



	bool IS_INITIALIZED = false;

	ShaftModel shaft_model = ShaftModel::GILSON_P200;



	int32_t VOLUME_FORMULA_OFFSET_NL;
	int32_t VOLUME_FORMULA_MULTIPLIER_NL;



	// Storage variables
	uint32_t serial_number = 0;

	int32_t ENCODER_POS_TOP		= 0;
	int32_t ENCODER_POS_INIT	= 0;
	int32_t ENCODER_POS_TIP		= 0;
	int32_t ENCODER_POS_BOTTOM	= 0;


	int32_t ENCODER_POS_TOP_SECURE		= 0;
	int32_t ENCODER_POS_INIT_SECURE		= 0;
	int32_t ENCODER_POS_TIP_SECURE		= 0;
	int32_t ENCODER_POS_BOTTOM_SECURE	= 0;



	//int32_t ENCODER_POS_INIT_MIN = 0;
	//int32_t ENCODER_POS_INIT_MAX = 0;


	// Encoder position
	int32_t SECURITY_OFFSET_ENCODER_POS = distance_to_encoder_pos(0.0005);	// 500 micrometers

	int32_t OFFSET_ENCODER_POS = 0;

	int32_t ENCODER_POS_MAX_ERROR_TOLERATED = 100; // +- 50 encoder pulse


	// Disable interrupt handler by default when generating a waveform
	bool DISABLE_TRAJECTORY_SYSTICK = true;
	bool DISABLE_TRAJECTORY_USBSERIAL = true;





	static const uint32_t TRAJECTORY_BUF_LEN = 5000;
	uint32_t trajectory_buf[TRAJECTORY_BUF_LEN];

	// Configuration


	uint32_t EJECT_MICROSTEP = pt_conf.eject_microstep;
	uint32_t HOMING_MICROSTEP= pt_conf.homing_microstep;
	uint32_t DEFAULT_MICROSTEP = pt_conf.default_microstep;

	double EJECT_SPEED 	= pt_conf.eject_speed;	// (m/s)
	double HOMING_SPEED = pt_conf.homing_speed;	// (m/s)

	double MAX_ACC 	= pt_conf.max_acc;		// (m/s^2)
	double MAX_START_SPEED  = pt_conf.max_start_speed;

	double DEFAULT_START_SPEED  = pt_conf.default_start_speed;


	int32_t prev_encoder_count = 0;
};






#else
#error Arduino model not supported
#endif // ARDUINO_SAMD_MKRZERO
#endif // ARDUINO



#endif /* PIPETTE_TOOL_H_ */
