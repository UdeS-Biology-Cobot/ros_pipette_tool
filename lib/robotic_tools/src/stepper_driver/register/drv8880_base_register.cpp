/*
 * drv8880_base_register.cpp
 *
 *  Created on: Jun 22, 2018
 *      Author: robotic_tools
 */


#include <robotic_tools/stepper_driver/register/drv8880_base_register.h>
#include <robotic_tools/error_handle/error_handle.h>


DRV8880BaseRegister::DRV8880BaseRegister()
{}
DRV8880BaseRegister::~DRV8880BaseRegister()
{}

DRV8880::MicrostepMode DRV8880BaseRegister::decode_microstep_mode(uint32_t data) {

	uint8_t _mres_bin = (data & MRES_BM) >> MRES_BP;

	MicrostepModeBinary mres_bin = MicrostepModeBinary(_mres_bin);

	DRV8880::MicrostepMode mres;

	switch (mres_bin)
	{
		case MicrostepModeBinary::FULL_STEP:
			mres = DRV8880::MicrostepMode::FULL_STEP;
			break;

		case MicrostepModeBinary::HALF_STEP:
			mres = DRV8880::MicrostepMode::HALF_STEP;
			break;

		case MicrostepModeBinary::HALF_STEP_CIRCULAR:
			mres = DRV8880::MicrostepMode::HALF_STEP_CIRCULAR;
			break;

		case MicrostepModeBinary::QUARTER_STEP:
			mres = DRV8880::MicrostepMode::QUARTER_STEP;
			break;

		case MicrostepModeBinary::EIGHTH_STEP:
			mres = DRV8880::MicrostepMode::EIGHTH_STEP;
			break;

		case MicrostepModeBinary::SIXTEENTH_STEP:
			mres = DRV8880::MicrostepMode::SIXTEENTH_STEP;
			break;

		default:
			p_err->set(DRV8880RegisterErrCode::DECODE_MRES_NOT_IMPLEMENTED);
			break;
	}

	return mres;
}

uint32_t DRV8880BaseRegister::encode_microstep_mode(DRV8880::MicrostepMode mres) {

	MicrostepModeBinary mres_bin;

	switch (mres)
	{
		case DRV8880::MicrostepMode::FULL_STEP:
			mres_bin = MicrostepModeBinary::FULL_STEP;
			break;

		case DRV8880::MicrostepMode::HALF_STEP:
			mres_bin = MicrostepModeBinary::HALF_STEP;
			break;

		case DRV8880::MicrostepMode::HALF_STEP_CIRCULAR:
			mres_bin = MicrostepModeBinary::HALF_STEP_CIRCULAR;
			break;

		case DRV8880::MicrostepMode::QUARTER_STEP:
			mres_bin = MicrostepModeBinary::QUARTER_STEP;
			break;

		case DRV8880::MicrostepMode::EIGHTH_STEP:
			mres_bin = MicrostepModeBinary::EIGHTH_STEP;
			break;

		case DRV8880::MicrostepMode::SIXTEENTH_STEP:
			mres_bin = MicrostepModeBinary::SIXTEENTH_STEP;
			break;

		default:
			p_err->set(DRV8880RegisterErrCode::ENCODE_MRES_NOT_IMPLEMENTED);
			break;
	}

	return ((uint32_t)mres_bin) << MRES_BP;
}





DRV8880::Current DRV8880BaseRegister::decode_current(uint32_t data) {


	uint8_t _current_bin = (data & CURRENT_BM) >> CURRENT_BP;

	CurrentBinary current_bin = CurrentBinary(_current_bin);

	DRV8880::Current current;


	switch (current_bin)
	{
		case CurrentBinary::PERCENT_25:
			current = DRV8880::Current::PERCENT_25;
			break;

		case CurrentBinary::PERCENT_50:
			current = DRV8880::Current::PERCENT_50;
			break;

		case CurrentBinary::PERCENT_75:
			current = DRV8880::Current::PERCENT_75;
			break;

		case CurrentBinary::PERCENT_100:
			current = DRV8880::Current::PERCENT_100;
			break;

		default:
			p_err->set(DRV8880RegisterErrCode::DECODE_CURRENT_NOT_IMPLEMENTED);
			break;
	}

	return current;
}

uint32_t DRV8880BaseRegister::encode_current(DRV8880::Current current) {

	CurrentBinary current_bin;

	switch (current)
	{
		case DRV8880::Current::PERCENT_25:
			current_bin = CurrentBinary::PERCENT_25;
			break;

		case DRV8880::Current::PERCENT_50:
			current_bin = CurrentBinary::PERCENT_50;
			break;

		case DRV8880::Current::PERCENT_75:
			current_bin = CurrentBinary::PERCENT_75;
			break;

		case DRV8880::Current::PERCENT_100:
			current_bin = CurrentBinary::PERCENT_100;
			break;

		default:
			p_err->set(DRV8880RegisterErrCode::ENCODE_CURRENT_NOT_IMPLEMENTED);
			break;
	}

	return ((uint32_t)current_bin) << CURRENT_BP;
}

DRV8880::Direction DRV8880BaseRegister::decode_direction(uint32_t data) {

	uint8_t _direction_bin = (data & DIRECTION_BM) >> DIRECTION_BP;

	DirectionBinary direction_bin = DirectionBinary(_direction_bin);

	DRV8880::Direction direction;


	switch (direction_bin)
	{
		case DirectionBinary::CLOCKWISE:
			direction = DRV8880::Direction::CLOCKWISE;
			break;

		case DirectionBinary::COUNTER_CLOCKWISE:
			direction = DRV8880::Direction::COUNTER_CLOCKWISE;
			break;

		default:
			p_err->set(DRV8880RegisterErrCode::DECODE_DIRECTION_NOT_IMPLEMENTED);
			break;
	}

	return direction;
}

uint32_t DRV8880BaseRegister::encode_direction(DRV8880::Direction direction) {

	DirectionBinary direction_bin;

	switch (direction)
	{
		case DRV8880::Direction::CLOCKWISE:
			direction_bin = DirectionBinary::CLOCKWISE;
			break;

		case DRV8880::Direction::COUNTER_CLOCKWISE:
			direction_bin = DirectionBinary::COUNTER_CLOCKWISE;
			break;

		default:
			p_err->set(DRV8880RegisterErrCode::ENCODE_DIRECTION_NOT_IMPLEMENTED);
			break;
	}

	return ((uint32_t)direction_bin) << DIRECTION_BP;
}

uint32_t DRV8880BaseRegister::encode_microstep_mode(DRV8880::MicrostepMode mres, uint32_t data)
{
	data = (data & ~MRES_BM);
	data |= encode_microstep_mode(mres);
	return data;
}

uint32_t DRV8880BaseRegister::encode_current(DRV8880::Current current, uint32_t data)
{
	data = (data & ~CURRENT_BM);
	data |= encode_current(current);
	return data;
}

uint32_t DRV8880BaseRegister::encode_direction(DRV8880::Direction direction, uint32_t data)
{
	data = (data & ~DIRECTION_BM);
	data |= encode_direction(direction);
	return data;
}
/*
uint16_t DRV8880BaseRegister::decode_microsteps(DRV8880::Mres mres)
{
	switch (mres)
	{
		case DRV8880::Mres::FULL_STEP:
			mres_bin = MresBinary::FULL_STEP;
			break;

		case DRV8880::Mres::HALF_STEP:
			mres_bin = MresBinary::HALF_STEP;
			break;

		case DRV8880::Mres::HALF_STEP_CIRCULAR:
			mres_bin = MresBinary::HALF_STEP_CIRCULAR;
			break;

		case DRV8880::Mres::QUARTER_STEP:
			mres_bin = MresBinary::QUARTER_STEP;
			break;

		case DRV8880::Mres::EIGHTH_STEP:
			mres_bin = MresBinary::EIGHTH_STEP;
			break;

		case DRV8880::Mres::SIXTEENTH_STEP:
			mres_bin = MresBinary::SIXTEENTH_STEP;
			break;

		default:
			p_err->set(DRV8880RegisterErrCode::ENCODE_MRES_NOT_IMPLEMENTED);
			break;
}
*/
