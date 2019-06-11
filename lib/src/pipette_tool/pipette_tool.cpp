/*
 * pipette_tool.cpp
 *
 *  Created on: May 24, 2018
 *      Author: biobot
 */
#if defined(ARDUINO)

#include <Arduino.h>

#include <robotic_tool/pipette_tool/pipette_tool.h>
//#include "pipette_tool.h"

/*
PipetteTool::PipetteTool()
{}
*/



PipetteTool::PipetteTool(StepperMotor*  motor,
			       	   	    Leadscrew*  leadscrew,
					  DriverInterface*  driver,
					 EncoderInterface*  encoder,
							  DRV5023*  he_sensor,
					    MotionProfile*  profile,
				   WaveformGeneration*  wf_gen,
				   	     const PtConf   pt_conf,
				       PipetteToolErr*  p_err)
: motor(motor),
  leadscrew(leadscrew),
  driver(driver),
  encoder(encoder),
  he_sensor(he_sensor),
  profile(profile),
  wf_gen(wf_gen),
  pt_conf(pt_conf)

{
}


void PipetteTool::init(PipetteToolErr* p_err)
{
	PipetteToolStorageErr err_storage = PipetteToolStorageErr::NONE;
	//EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;

	// Storage
	storage.retrieve_data(PipetteToolStorage::DataStorage::PT_SN, &serial_number, &err_storage);
	if(storage.err_get(&err_storage)) {
		*p_err = PipetteToolErr::STORAGE_ERROR;
		return;
	}


	storage.retrieve_data(PipetteToolStorage::DataStorage::EP_TOP, &ENCODER_POS_TOP, &err_storage);
	if(storage.err_get(&err_storage)) {
		*p_err = PipetteToolErr::STORAGE_ERROR;
		return;
	}

/*
	storage.retrieve_data(PipetteToolStorage::DataStorage::EP_INIT, &ENCODER_POS_INIT, &err_storage);
	if(storage.err_get(&err_storage)) {
		*p_err = PipetteToolErr::STORAGE_ERROR;
		return;
	}
*/
	storage.retrieve_data(PipetteToolStorage::DataStorage::EP_TIP, &ENCODER_POS_TIP, &err_storage);
	if(storage.err_get(&err_storage)) {
		*p_err = PipetteToolErr::STORAGE_ERROR;
		return;
	}
	storage.retrieve_data(PipetteToolStorage::DataStorage::EP_BOT, &ENCODER_POS_BOTTOM, &err_storage);
	if(storage.err_get(&err_storage)) {
		*p_err = PipetteToolErr::STORAGE_ERROR;
		return;
	}


	set_volume_formula(shaft_model, p_err);
	if (err_get(p_err)) {return;}
	/*
	storage.retrieve_data(PipetteToolStorage::DataStorage::EP_MIN, &ENCODER_POS_INIT_MIN, &err_storage);
	if(storage.err_get(&err_storage)) {
		*p_err = PipetteToolErr::STORAGE_ERROR;
		return;
	}
	storage.retrieve_data(PipetteToolStorage::DataStorage::EP_MAX, &ENCODER_POS_INIT_MAX, &err_storage);
	if(storage.err_get(&err_storage)) {
		*p_err = PipetteToolErr::STORAGE_ERROR;
		return;
	}
	 */


	//Serial.println("do homing");
	// Homing
	do_homing(p_err);
	if (err_get(p_err)) {return;}

	// Rearange encoder position so that the top value is an absolute 0
	calc_encoder_pos_offset();


	IS_INITIALIZED = true;
}


uint32_t PipetteTool::get_serial_number(PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return 0;}

	//Serial.print("serial_number = ");
	//Serial.println(serial_number);
	return serial_number;
}


void PipetteTool::aspirate(uint32_t nL, double speed, PipetteToolErr* p_err)
{
	//Serial.println(speed, 32);

	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return;}

	move(nL, speed, DisplacementDirection::ASPIRATE, p_err);
	if(err_get(p_err)) {return;}
}

void PipetteTool::dispense(uint32_t nL, double speed, PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return;}

	move(nL, speed, DisplacementDirection::DISPENSE, p_err);
	if(err_get(p_err)) {return;}
}



void PipetteTool::move_to_top(PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return;}

	move_to_encoder_pos(ENCODER_POS_TOP_SECURE, 0.03, 16, p_err);
}


void PipetteTool::move_to_tip(PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return;}

	move_to_encoder_pos(ENCODER_POS_TIP_SECURE, 0.03, 16, p_err);
}


void PipetteTool::move_to_bottom(PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return;}

	move_to_encoder_pos(ENCODER_POS_BOTTOM_SECURE, 0.03, 16, p_err);
}







void PipetteTool::move_to_encoder_pos(int32_t encoder_pos, double speed, uint32_t microstep, PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return;}

	DriverInterfaceErr err_driver = DriverInterfaceErr::NONE;
	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;
	DisplacementDirection dir;

	// get current position
	int32_t curr_pos = encoder->read(OFFSET_ENCODER_POS, &err_encoder);
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return;
	}

	// Already at the right position
	if (encoder_pos - curr_pos == 0) {
		return;
	}

	// Check min and max position allowed
	if (encoder_pos < ENCODER_POS_TOP_SECURE || encoder_pos > ENCODER_POS_BOTTOM_SECURE) {
		*p_err = PipetteToolErr::OUT_OF_BOND;
		return;
	}

	prev_encoder_count = curr_pos;

	// Set driver microstep
	driver->set_microstep(microstep, &err_driver);
	if (driver->err_get(&err_driver)) {
		*p_err = PipetteToolErr::DRIVER_ERROR;
		return;
	}

	// Select the right direction for stepper driver
	if (encoder_pos - curr_pos > 0) {
		dir = DisplacementDirection::DISPENSE;
	} else {
		dir = DisplacementDirection::ASPIRATE;
	}

	set_displacement_dir(dir, p_err);
	if(err_get(p_err)) {return;}


	int32_t encoder_pos_diff = abs(encoder_pos - curr_pos);
	uint32_t mstep = encoder_pos_to_mstep(encoder_pos_diff, microstep);

	Serial.print("mstep");
	Serial.println(mstep);

	// Generate trajectory
	generate_trajectory(speed, mstep, microstep, p_err);
	if (err_get(p_err)) {
		*p_err = PipetteToolErr::TRAJECTORY_ERROR;
		return;
	}

	validate_trajectory(mstep, curr_pos, microstep, p_err);
	if(err_get(p_err)) {return;}
}





void PipetteTool::move(uint32_t nl, double speed, DisplacementDirection dir, PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return;}

	DriverInterfaceErr err_driver = DriverInterfaceErr::NONE;
	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;

	uint32_t microstep = DEFAULT_MICROSTEP;


	// Set driver microstep
	driver->set_microstep(microstep, &err_driver);
	if (driver->err_get(&err_driver)) {
		*p_err = PipetteToolErr::DRIVER_ERROR;
		return;
	}

	// Set direction
	set_displacement_dir(dir, p_err);
	if(err_get(p_err)) {return;}

	// get current position
	int32_t curr_pos = encoder->read(OFFSET_ENCODER_POS, &err_encoder);
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return;
	}


	prev_encoder_count = curr_pos;

	// Convert nl to steps
	uint32_t mstep = nl_to_mstep(nl, microstep, p_err);
	uint32_t encoder_count = mstep_to_encoder_pos(mstep, microstep);

	//delay(5000);
	Serial.print("nl = ");
	Serial.println(nl);
	Serial.print("curr_pos = ");
	Serial.println(curr_pos);
	Serial.print("mstep = ");
	Serial.println(mstep);
	Serial.print("encoder_count = ");
	Serial.println(encoder_count);
	Serial.print("ENCODER_POS_TOP_SECURE = ");
	Serial.println(ENCODER_POS_TOP_SECURE);
	Serial.print("ENCODER_POS_TIP_SECURE = ");
	Serial.println(ENCODER_POS_TIP_SECURE);


	// validate that the steps can be made
	if (dir == DisplacementDirection::ASPIRATE) {
		if ((curr_pos - (int32_t)encoder_count) < ENCODER_POS_TOP_SECURE - ENCODER_POS_MAX_ERROR_TOLERATED) {
			Serial.println("Out of bound");
			*p_err = PipetteToolErr::OUT_OF_BOND;
			return;
		}
	} else {
		if ((int32_t)(curr_pos + encoder_count) >= ENCODER_POS_TIP_SECURE + ENCODER_POS_MAX_ERROR_TOLERATED) {
			*p_err = PipetteToolErr::OUT_OF_BOND;
			return;
		}
	}

	// Generate trajectory
	generate_trajectory(speed, mstep, microstep, p_err);
	if (err_get(p_err)) {
		*p_err = PipetteToolErr::TRAJECTORY_ERROR;
		return;
	}


	validate_trajectory(mstep, curr_pos, microstep, p_err);
	if(err_get(p_err)) {return;}


}

void PipetteTool::multiple_sequence(uint32_t nl, double speed, DisplacementDirection start_dir, uint32_t seq_number, PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return;}

/*
	Serial.print("nl = ");
	Serial.println(nl);
	Serial.print("speed");
	Serial.println(speed);
	Serial.print("start_dir = ");
	Serial.println((bool)start_dir);
	Serial.print("seq_number = ");
	Serial.println(seq_number);
*/

	DriverInterfaceErr err_driver = DriverInterfaceErr::NONE;
	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;

	uint32_t microstep = DEFAULT_MICROSTEP;


	// Set driver microstep
	driver->set_microstep(microstep, &err_driver);
	if (driver->err_get(&err_driver)) {
		*p_err = PipetteToolErr::DRIVER_ERROR;
		return;
	}




	// get current position
	int32_t curr_pos = encoder->read(OFFSET_ENCODER_POS, &err_encoder);
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return;
	}


	// Convert nl to steps
	uint32_t mstep = nl_to_mstep(nl, microstep, p_err);
	int32_t encoder_count = mstep_to_encoder_pos(mstep, microstep);

	// Validate for out of bounds positions
	if (start_dir == DisplacementDirection::ASPIRATE) {

		// Validate for Aspirate
		if ((curr_pos - (int32_t)encoder_count) < ENCODER_POS_TOP_SECURE - ENCODER_POS_MAX_ERROR_TOLERATED) {
			*p_err = PipetteToolErr::OUT_OF_BOND;
			return;
		}

		// Validate for Dispense
		if((curr_pos - (int32_t)encoder_count) + encoder_count > ENCODER_POS_TIP_SECURE + ENCODER_POS_MAX_ERROR_TOLERATED) {
			*p_err = PipetteToolErr::OUT_OF_BOND;
			return;
		}

	} else {
		// Validate for Dispense
		if ((int32_t)(curr_pos + encoder_count) > ENCODER_POS_TIP_SECURE + ENCODER_POS_MAX_ERROR_TOLERATED) {
			*p_err = PipetteToolErr::OUT_OF_BOND;
			return;
		}
		// Validate for Aspirate
		if((curr_pos + (int32_t)encoder_count) - encoder_count < ENCODER_POS_TOP_SECURE - ENCODER_POS_MAX_ERROR_TOLERATED) {
			*p_err = PipetteToolErr::OUT_OF_BOND;
			return;
		}
	}

	// Generate profile
	bm_compact bm = generate_profile(speed, mstep, microstep, p_err);
	if(err_get(p_err)) {return;}



	for (uint32_t i = 0; i < seq_number; i++) {
		// Set direction
		set_displacement_dir(start_dir, p_err);
		if(err_get(p_err)) {return;}

		// Generate waveform
		generate_waveform(bm, p_err);
		if(err_get(p_err)) {return;}

		// Validate waveform
		curr_pos = validate_trajectory(mstep, curr_pos, microstep, p_err);
		if(err_get(p_err)) {return;}

		if (i == seq_number - 2) {
			prev_encoder_count = curr_pos;
		}


		// Toggle direction
		if (start_dir == DisplacementDirection::ASPIRATE) {
			start_dir = DisplacementDirection::DISPENSE;
		} else {
			start_dir = DisplacementDirection::ASPIRATE;
		}

	}
}



uint32_t PipetteTool::get_max_nl(PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return 0;}

	uint32_t max_distance_encoder_pos = ENCODER_POS_TIP_SECURE - ENCODER_POS_TOP_SECURE;
	double max_distance = encoder_pos_to_distance(max_distance_encoder_pos);

	uint32_t nl = m_to_nl(max_distance, p_err);
	if(err_get(p_err)) {return 0;}

	Serial.print("max_distance_encoder_pos = ");
	Serial.println(max_distance_encoder_pos);
	Serial.print("max_distance = ");
	Serial.println(max_distance, 16);
	Serial.print("max nl = ");
	Serial.println(nl);

	//nl -= 1918;

	return nl;
}

uint32_t PipetteTool::nl_to_mstep(uint32_t nL, uint32_t microstep, PipetteToolErr* p_err)
{
	double distance = nl_to_m(nL, p_err);
	if(err_get(p_err)) {return 0;}

	uint32_t mstep = m_to_mstep(distance, microstep);
	return mstep;
}



double PipetteTool::nl_to_m(int32_t nL, PipetteToolErr* p_err)
{
	double distance = nl_to_mm(nL, p_err);
	if(err_get(p_err)) {return 0.0;}

	// translate to meters
	distance /= 1000;
	return distance;
}


int32_t PipetteTool::m_to_nl(double distance, PipetteToolErr* p_err)
{


	int32_t nl = mm_to_nl(distance * (double)1000.0, p_err);
	if(err_get(p_err)) {return 0;}

	return nl;
}


int32_t PipetteTool::mm_to_nl(double distance, PipetteToolErr* p_err)
{
	// y = ax - b
	// nL = a*distance(mm) + (b)
	// distance(mm) = (nL - b) / a

	//uint32_t a;
	//uint32_t b;
	int32_t nl;


	/*
	switch (shaft_model) {

		case ShaftModel::GILSON_P200:
			a = 12499;
			b = 1918;
			break;

		default:
			*p_err = PipetteToolErr::INVALID_SHAFT_MODEL;
			return 0.0;
	}
*/


	nl = ((double)VOLUME_FORMULA_MULTIPLIER_NL * distance) + (double)VOLUME_FORMULA_OFFSET_NL;
	return nl;
}








double PipetteTool::nl_to_mm(int32_t nL, PipetteToolErr* p_err)
{
	// y = ax - b
	// nL = a*distance(mm) - b
	// distance(mm) = (nL + b) / a
	//uint32_t a;
	//uint32_t b;
	double distance_mm;
/*
	switch (shaft_model) {

		case ShaftModel::GILSON_P200:
			a = 12499;
			b = 1918;
			break;

		default:
			*p_err = PipetteToolErr::INVALID_SHAFT_MODEL;
			return 0.0;
	}
*/
	distance_mm = ((double)nL - (double)VOLUME_FORMULA_OFFSET_NL) / (double)VOLUME_FORMULA_MULTIPLIER_NL;
	return distance_mm;
}


uint32_t PipetteTool::m_to_mstep(double distance, uint32_t microstep)
{
	uint32_t msteps = distance / one_mstep_to_distance(microstep);
	return msteps;
}


uint32_t PipetteTool::mstep_to_encoder_pos(uint32_t msteps, uint32_t microstep)
{
	// get distance
	double step_distance = one_mstep_to_distance(microstep);	//
	double distance = msteps*step_distance;

	uint32_t encoder_count = distance * (double)encoder->get_resolution() / leadscrew->lead;
	return encoder_count;
}



uint32_t PipetteTool::encoder_pos_to_mstep(int32_t encoder_count ,uint32_t microstep)
{
	double step_distance = one_mstep_to_distance(microstep);

	double distance_to_travel = encoder_pos_to_distance(encoder_count);

	uint32_t steps = distance_to_travel / step_distance;
	return steps;
}

double PipetteTool::one_mstep_to_distance(uint32_t microstep)
{
	return leadscrew->lead / ((double)motor->steps_per_rev * (double)microstep);
}

double PipetteTool::encoder_pos_to_distance(int32_t encoder_pos)
{
	return (double)encoder_pos * leadscrew->lead / (double)encoder->get_resolution();
}

uint32_t PipetteTool::distance_to_encoder_pos(double distance)
{
	uint32_t encoder_pos = distance *(double)encoder->get_resolution() / leadscrew->lead;	// 0.0002 = 200 micromiters
	return encoder_pos;
}



void PipetteTool::eject_tip(PipetteToolErr* p_err)
{
	if (!IS_INITIALIZED) {*p_err = PipetteToolErr::NOT_INITIALIZED; return;}


	// Set error
	//MotionProfileErr err_profile = MotionProfileErr::NONE;
	//WaveformGenerationErr err_wf_gen = WaveformGenerationErr::NONE;
	DriverInterfaceErr err_driver = DriverInterfaceErr::NONE;
	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;

	uint32_t microstep = EJECT_MICROSTEP;


	// get current position
	int32_t curr_pos = encoder->read(OFFSET_ENCODER_POS, &err_encoder);
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return;
	}


	// If already in the eject zone skip
	if (curr_pos >= ENCODER_POS_BOTTOM_SECURE) {
		return;
	}

	prev_encoder_count = curr_pos;

	set_displacement_dir(DisplacementDirection::DISPENSE, p_err);
	if(err_get(p_err)) {return;}

	// Set driver microstep
	driver->set_microstep(microstep, &err_driver);
	if (driver->err_get(&err_driver)) {
		*p_err = PipetteToolErr::DRIVER_ERROR;
		return;
	}

	uint32_t encoder_count_diff = ENCODER_POS_BOTTOM_SECURE - curr_pos;
	uint32_t mstep = encoder_pos_to_mstep(encoder_count_diff, microstep);

	generate_trajectory2(EJECT_SPEED, mstep, microstep, p_err);
	if (err_get(p_err)) {
		*p_err = PipetteToolErr::TRAJECTORY_ERROR;
		return;
	}


	validate_trajectory(mstep, curr_pos, microstep, p_err);
	if(err_get(p_err)) {return;}



	// TODO remove just for test
	//delay(5000);
	// Compare
	/*
	int32_t pos = encoder->read(OFFSET_ENCODER_POS, &err_encoder);
	Serial.print("Encoder end pos = ");
	Serial.println(pos);
	Serial.print("ENCODER_POS_BOTTOM_SECURE = ");
	Serial.println(ENCODER_POS_BOTTOM_SECURE);
	Serial.print("ENCODER_POS_BOTTOM = ");
	Serial.println(ENCODER_POS_BOTTOM+OFFSET_ENCODER_POS);
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return;
	}
*/
}





void PipetteTool::calc_encoder_pos_offset()
{
	// Add security offset
	ENCODER_POS_TOP_SECURE = ENCODER_POS_TOP + SECURITY_OFFSET_ENCODER_POS;

	// TOP position will now be our position 0
	OFFSET_ENCODER_POS = -ENCODER_POS_TOP_SECURE;

	// Change to custom absolute
	ENCODER_POS_TOP_SECURE	   += OFFSET_ENCODER_POS;
	ENCODER_POS_TIP_SECURE 	 	= ENCODER_POS_TIP + OFFSET_ENCODER_POS - SECURITY_OFFSET_ENCODER_POS;
	ENCODER_POS_BOTTOM_SECURE   = ENCODER_POS_BOTTOM + OFFSET_ENCODER_POS -SECURITY_OFFSET_ENCODER_POS;


	Serial.print("ENCODER_POS_TOP_SECURE = ");
	Serial.println(ENCODER_POS_TOP_SECURE);
	Serial.print("ENCODER_POS_TIP_SECURE = ");
	Serial.println(ENCODER_POS_TIP_SECURE);
	Serial.print("ENCODER_POS_BOTTOM_SECURE = ");
	Serial.println(ENCODER_POS_BOTTOM_SECURE);

}


void PipetteTool::set_displacement_dir(DisplacementDirection dir, PipetteToolErr* p_err)
{
	Leadscrew::Handedness handedness = leadscrew->handedness;
	DriverInterfaceErr err_driver = DriverInterfaceErr::NONE;


	if (dir == PipetteTool::DisplacementDirection::DISPENSE) {
		if (handedness == Leadscrew::Handedness::RIGHT_HANDED) {
			driver->set_direction(driver->Direction::CW, &err_driver);
		}
		else {
			driver->set_direction(driver->Direction::CCW, &err_driver);
		}
	}
	else {
		if (handedness == Leadscrew::Handedness::RIGHT_HANDED) {
			driver->set_direction(driver->Direction::CCW, &err_driver);
		}
		else {
			driver->set_direction(driver->Direction::CW, &err_driver);
		}
	}

	if (driver->err_get(&err_driver)) {
		*p_err = PipetteToolErr::DRIVER_ERROR;
		Serial.println("pokiki");
	}


	//delay(100);

}




void PipetteTool::do_homing(PipetteToolErr* p_err)
{
	// No need to be initialized

	DRV5023Err drv5023_err = DRV5023Err::NONE;
	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;

	// Move out of trigered position
	if(he_sensor->is_triggered(&drv5023_err)) {
		if (he_sensor->err_get(&drv5023_err)) {
			*p_err = PipetteToolErr::HALL_EFFECT_SENSOR_ERROR;
			return;
		}

		set_displacement_dir(DisplacementDirection::DISPENSE, p_err);
		if (err_get(p_err)) {return;}

		homing_sequence(RISING, p_err);
		if (err_get(p_err)) {return;}
	}

	if (he_sensor->err_get(&drv5023_err)) {
		*p_err = PipetteToolErr::HALL_EFFECT_SENSOR_ERROR;
		return;
	}

	// Initialise
	set_displacement_dir(DisplacementDirection::ASPIRATE, p_err);
	if (err_get(p_err)) {return;}

	homing_sequence(FALLING, p_err);
	if (err_get(p_err)) {return;}


	encoder->reset(&err_encoder);
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return;
	}





	if (IS_INITIALIZED) {
		/*
		// TODO check if necessary
		// Check that the absolute encoder is within MIN and MAX init range
		if (pos < ENCODER_POS_INIT_MIN || pos > ENCODER_POS_INIT_MAX) {
			*p_err = PipetteToolErr::HOMING_ERROR;
			Serial.print("Encoder pos = ");
			Serial.println(pos);
			return;
		}
		Serial.print("Encoder pos = ");
		Serial.println(pos);
		*/
	}



}


void PipetteTool::homing_sequence(uint32_t trigger, PipetteToolErr* p_err) {
	// Set error
	DriverInterfaceErr err_driver = DriverInterfaceErr::NONE;

	uint32_t microstep = HOMING_MICROSTEP;
	double speed = HOMING_SPEED;
	uint32_t mstep = UINT32_MAX;	// Set to max because of unknown position


	he_sensor->enable_interrupt(wf_gen->stop_generation, trigger);

	// Set driver microstep to the max
	driver->set_microstep(microstep, &err_driver);
	if (driver->err_get(&err_driver)) {
		*p_err = PipetteToolErr::DRIVER_ERROR;
		goto error;
	}

	generate_trajectory(speed, mstep, microstep, p_err);
	if (*p_err != PipetteToolErr::EMERGENCY_STOP) {
		*p_err = PipetteToolErr::TRAJECTORY_ERROR;
		goto error;
	}
	else {
		err_clear(p_err);	// reset error
	}

	he_sensor->disable_interrupt();
	return;

error:
	he_sensor->disable_interrupt();

}




bm_compact PipetteTool::generate_profile(double speed, const uint32_t steps, uint32_t microstep, PipetteToolErr* p_err)
{
	MotionProfileErr err_profile = MotionProfileErr::NONE;
	WaveformGenerationErr err_wf_gen = WaveformGenerationErr::NONE;

	uint32_t f_min;
	uint32_t f_max;
	uint32_t slope;

	// Trapezoidal velocity
	if (speed > MAX_START_SPEED) {
		f_min = speed_to_freq(DEFAULT_START_SPEED, microstep);
		Serial.print("f_min = ");
		Serial .println(f_min);
		f_max = speed_to_freq(speed, microstep);
		Serial.print("f_max = ");
		Serial .println(f_max);
		slope = acc_to_freq(speed, MAX_ACC, microstep);

		Serial.print("Slope = ");
		Serial .println(slope);
		Serial.print("Steps = ");
		Serial .println(steps);
	}
	// Constant velocity
	else {
		f_min = speed_to_freq(speed, microstep);
		Serial.print("f_min = ");
		Serial.println(f_min);
		f_max = f_min;
		slope = 0;
	}

	bm_compact bm;	// Buffer Mode compact


	// Compute trajectory -> buf contains rampup frequencies and bm contains information
	// about rampup, plateau and rampdown index and length + buffer length used
	bm = profile->compute_frequency_vel(trajectory_buf, TRAJECTORY_BUF_LEN, steps, f_min, f_max, slope, &err_profile);
	if (profile->err_get(&err_profile)) {
		Serial.print("MOTION_PROFILE_ERROR = ");
		Serial.println((uint32_t)err_profile);
		*p_err = PipetteToolErr::MOTION_PROFILE_ERROR;
		return bm;
	}

	// Convert frequencies to match the low level waveform generator
	wf_gen->convert_frequencies(&bm, trajectory_buf, &err_wf_gen);
	if (wf_gen->err_get(&err_wf_gen)) {
		Serial.print("WAVEFORM_GENERATION_ERROR = ");
		Serial.println((uint32_t)err_wf_gen);
		*p_err = PipetteToolErr::WAVEFORM_GENERATION_ERROR;
		return bm;
	}


	return bm;
}


void PipetteTool::generate_waveform(const bm_compact bm, PipetteToolErr* p_err)
{
	WaveformGenerationErr err_wf_gen = WaveformGenerationErr::NONE;

	// Generate waveform
	wf_gen->gen_dual_slope_pwm(bm, trajectory_buf, &err_wf_gen, DISABLE_TRAJECTORY_SYSTICK, DISABLE_TRAJECTORY_USBSERIAL);

	if (wf_gen->err_get(&err_wf_gen)) {
		//
		if (err_wf_gen == WaveformGenerationErr::PULSE_MISMATCH) {
			Serial.println("PULSE_MISMATCH");
			*p_err = PipetteToolErr::EMERGENCY_STOP;
			return;
		}
		else {
			Serial.print("WAVEFORM_GENERATION_ERROR = ");
			Serial.println((uint32_t)err_wf_gen);
			//Serial .println("WAVEFORM_GENERATION_ERROR");
			*p_err = PipetteToolErr::WAVEFORM_GENERATION_ERROR;
			return;
		}
	}
}






uint32_t PipetteTool::get_rem_dispense_vol_nl(PipetteToolErr* p_err)
{

	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;


	int32_t curr_pos = encoder->read(OFFSET_ENCODER_POS , &err_encoder);
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return 0;
	}


	int32_t rem_pos = ENCODER_POS_TIP_SECURE - curr_pos;

	// Return 0 nl if lower then tip
	if (rem_pos < 0 ) {
		return 0;
	}

	double distance = encoder_pos_to_distance(rem_pos);

	int32_t nl = m_to_nl(distance, p_err);
	if(err_get(p_err)) {return 0;}

	if (nl < 0) {
		return 0;
	}

	return nl;
}

uint32_t PipetteTool::get_rem_aspirate_vol_nl(PipetteToolErr* p_err)
{

	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;







	int32_t curr_pos = encoder->read(OFFSET_ENCODER_POS , &err_encoder);
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return 0;
	}


	//uint32_t rem_pos = abs(curr_pos - ENCODER_POS_TOP_SECURE);

	int32_t rem_pos = curr_pos - ENCODER_POS_TOP_SECURE;

	// Return 0 nl if greater then top
	if (rem_pos < 0 ) {
		return 0;
	}

	double distance = encoder_pos_to_distance(rem_pos);

	int32_t nl = m_to_nl(distance, p_err);
	if(err_get(p_err)) {return 0;}

	if (nl < 0) {
		return 0;
	}



	return nl;
}





/*
int32_t PipetteTool::get_nl_pos(PipetteToolErr* p_err)
{
	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;

	//
	int32_t curr_pos = encoder->read(OFFSET_ENCODER_POS , &err_encoder);

	Serial.println("################################");
	Serial.print("curr pos = ");
	Serial.println(curr_pos);
	// get current position
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return 0;
	}




	//- ENCODER_POS_TIP_SECURE
	//prev_encoder_count = curr_pos;
	int32_t abs_curr_pos = abs((curr_pos - prev_encoder_count) - ENCODER_POS_TIP_SECURE);
	double distance = encoder_pos_to_distance(abs_curr_pos);

	uint32_t nl = m_to_nl(distance, p_err);
	if(err_get(p_err)) {return 0;}


	int32_t _nl = nl;
	if (curr_pos >= 0) {
		_nl *= -1;
	}

	return _nl;


}
*/
double PipetteTool::get_max_speed()
{
	float motor_max_rps = motor->max_vel_rps;

	double max_speed = (double)motor_max_rps * leadscrew->lead;
	return max_speed;
}

int32_t PipetteTool::get_offset_nl()
{
	return VOLUME_FORMULA_OFFSET_NL;
}

void PipetteTool::set_volume_formula(ShaftModel _shaft_model, PipetteToolErr* p_err)
{
	switch (_shaft_model) {

		case ShaftModel::GILSON_P200:
			VOLUME_FORMULA_MULTIPLIER_NL = 12499;
			VOLUME_FORMULA_OFFSET_NL = -1918;
			break;

		default:
			*p_err = PipetteToolErr::INVALID_SHAFT_MODEL;
			return;
	}

}


void PipetteTool::generate_trajectory(double speed, const uint32_t mstep, uint32_t microstep, PipetteToolErr* p_err)
{
	if (mstep > 0) {
		// Generate profile
		bm_compact bm = generate_profile(speed, mstep, microstep, p_err);
		if(err_get(p_err)) {
			Serial.println("generate_profile");
			return;
		}

		// Generate waveform
		generate_waveform(bm, p_err);
		if(err_get(p_err)) {
			Serial.println("generate_waveform");
			return;
		}
	} else {
		Serial.println("Guard against zero");
	}
}


void PipetteTool::generate_trajectory2(double speed, const uint32_t steps, uint32_t microstep, PipetteToolErr* p_err)
{
	MotionProfileErr err_profile = MotionProfileErr::NONE;
	WaveformGenerationErr err_wf_gen = WaveformGenerationErr::NONE;



	double d_step = one_mstep_to_distance(microstep);
	double v_max = 0.03;
	double a_max = ((double)microstep / 64.0)*5.0;	// 5 -> 64


	bm_compact bm;	// Buffer Mode compact


	// Compute trajectory -> buf contains rampup frequencies and bm contains information
	// about rampup, plateau and rampdown index and length + buffer length used
	bm = profile->compute_trapezoidal_vel(trajectory_buf, TRAJECTORY_BUF_LEN, steps, d_step, v_max, a_max, &err_profile);
	if (profile->err_get(&err_profile)) {
		//delay(5000);
		//Serial.print("MOTION_PROFILE_ERROR code = ");
		//Serial.println((uint8_t)err_profile);
		*p_err = PipetteToolErr::MOTION_PROFILE_ERROR;
		return;
	}

	// Convert frequencies to match the low level waveform generator
	wf_gen->convert_frequencies(&bm, trajectory_buf, &err_wf_gen);
	if (wf_gen->err_get(&err_wf_gen)) {
		//delay(5000);
		//Serial.print("WAVEFORM_GENERATION_ERROR code = ");
		//Serial.println((uint8_t)err_wf_gen);
		*p_err = PipetteToolErr::WAVEFORM_GENERATION_ERROR;
		return;
	}

	// Generate waveform
	wf_gen->gen_dual_slope_pwm(bm, trajectory_buf, &err_wf_gen, DISABLE_TRAJECTORY_SYSTICK, DISABLE_TRAJECTORY_USBSERIAL);

	if (wf_gen->err_get(&err_wf_gen)) {
		//
		//delay(5000);
		if (err_wf_gen == WaveformGenerationErr::PULSE_MISMATCH) {
			//Serial .println("Emergency stop");
			*p_err = PipetteToolErr::EMERGENCY_STOP;
			return;
		}
		else {
			//Serial .println("WAVEFORM_GENERATION_ERROR");
			*p_err = PipetteToolErr::WAVEFORM_GENERATION_ERROR;
			return;
		}
	}
}






int32_t PipetteTool::validate_trajectory(uint32_t mstep, int32_t prev_pos, uint32_t microstep, PipetteToolErr* p_err)
{
	EncoderInterfaceErr err_encoder = EncoderInterfaceErr::NONE;

	// get current position
	int32_t curr_pos = encoder->read(OFFSET_ENCODER_POS, &err_encoder);
	if (encoder->err_get(&err_encoder)) {
		*p_err = PipetteToolErr::ENCODER_ERROR;
		return curr_pos;
	}

	if (curr_pos > ENCODER_POS_BOTTOM_SECURE + ENCODER_POS_MAX_ERROR_TOLERATED || curr_pos < ENCODER_POS_TOP_SECURE - ENCODER_POS_MAX_ERROR_TOLERATED) {
		*p_err = PipetteToolErr::OUT_OF_BOND;
		return curr_pos;
	}

	uint32_t distance_traveled = abs(curr_pos - prev_pos);

	uint32_t pos_mstep = mstep_to_encoder_pos(mstep, microstep);

	uint32_t pos_diff = abs((int32_t)pos_mstep - (int32_t)distance_traveled);


	//delay(5000);
	Serial.print("curr_pos = ");
	Serial.println(curr_pos);
	Serial.print("prev_pos = ");
	Serial.println(prev_pos);
	Serial.print("distance_traveled = ");
	Serial.println(distance_traveled);
	Serial.print("pos_mstep = ");
	Serial.println(pos_mstep);
	Serial.print("pos_diff = ");
	Serial.println(pos_diff);
	Serial.println("-----------------------------------------");

	if ((int32_t)pos_diff > ENCODER_POS_MAX_ERROR_TOLERATED) {
		*p_err = PipetteToolErr::POSITION_ERROR;
		return curr_pos;
	}

	return curr_pos;
}









uint32_t PipetteTool::speed_to_freq(double vel, uint32_t microstep)
{
	double step_distance = one_mstep_to_distance(microstep);
	uint32_t freq = vel / step_distance;

	return freq;
}



uint32_t PipetteTool::acc_to_freq(double vel, double acc, uint32_t microstep)
{
	double step_distance = one_mstep_to_distance(microstep);
	//Serial.print("step_distance = ");
	//Serial .println(step_distance);
	double freq_vel = vel / step_distance;
	//Serial.print("freq_vel = ");
	//Serial .println(freq_vel);

	double period = 1.0/freq_vel;
	//Serial.print("period = ");
	//Serial .println(period);

	uint32_t freq_acc = acc * freq_vel;
	return freq_acc / microstep;
}











void PipetteTool::disable_trajectory_systick()
{
	DISABLE_TRAJECTORY_SYSTICK = true;
}

void PipetteTool::disable_trajectory_usbserial()
{
	DISABLE_TRAJECTORY_USBSERIAL = true;
}















void PipetteTool::enable_trajectory_systick()
{
	DISABLE_TRAJECTORY_SYSTICK = false;
}

void PipetteTool::enable_trajectory_usbserial()
{
	DISABLE_TRAJECTORY_USBSERIAL = false;
}











bool PipetteTool::err_get(PipetteToolErr* p_err)
{
	if (*p_err == PipetteToolErr::NONE) {
		return false;
	}
	return true;
}

void PipetteTool::err_clear(PipetteToolErr* p_err)
{
	*p_err = PipetteToolErr::NONE;
}

#endif

