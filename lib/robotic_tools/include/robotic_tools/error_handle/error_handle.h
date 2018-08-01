/*
 * error_handle.h
 *
 *  Created on: May 24, 2018
 *      Author: robotic_tools
 */

#ifndef ERROR_HANDLE_H_
#define ERROR_HANDLE_H_

#include <stdint.h>

#include <robotic_tools/error_handle/error_code.h>


class ErrorHandle
{
public:
	bool flag = false;

	uint32_t err_code_offset = 65535;


	static const uint32_t OFFSET_TRAJECTORY_ERROR_CODE 						= 0x00;
	static const uint32_t OFFSET_TRAJECTORY_REGISTER_ERROR_CODE 			= OFFSET_TRAJECTORY_ERROR_CODE + (uint32_t)TrajectoryErrCode::ALWAYS_AT_END;
	static const uint32_t OFFSET_PROFILE_CONCAVE_SCURVE_ERROR_CODE			= OFFSET_TRAJECTORY_REGISTER_ERROR_CODE + (uint32_t)TrajectoryRegisterErrCode::ALWAYS_AT_END;
	static const uint32_t OFFSET_DRV8880_ERROR_CODE							= OFFSET_PROFILE_CONCAVE_SCURVE_ERROR_CODE + (uint32_t)ProfileConcaveScurveVelErrCode::ALWAYS_AT_END;
	static const uint32_t OFFSET_DRV8880_REGISTER_ERROR_CODE				= OFFSET_DRV8880_ERROR_CODE + (uint32_t)DRV8880ErrCode::ALWAYS_AT_END;
	static const uint32_t OFFSET_PWM_PFC_ERROR_CODE							= OFFSET_DRV8880_REGISTER_ERROR_CODE + (uint32_t)DRV8880RegisterErrCode::ALWAYS_AT_END;

	static const uint32_t OFFSET_DRV8880_MASTER_PROTOCOL_ERROR_CODE			= OFFSET_PWM_PFC_ERROR_CODE+ (uint32_t)PwmPfcErrCode::ALWAYS_AT_END;


	void set(TrajectoryErrCode err){
		err_code = (uint32_t)err;
		err_code_offset = err_code + OFFSET_TRAJECTORY_ERROR_CODE;
		flag = true;
	};

	void set(TrajectoryRegisterErrCode err){
		err_code = (uint32_t)err;
		err_code_offset = err_code + OFFSET_TRAJECTORY_REGISTER_ERROR_CODE;
		flag = true;
	};

	void set(ProfileConcaveScurveVelErrCode err){
		err_code = (uint32_t)err;
		err_code_offset = err_code + OFFSET_PROFILE_CONCAVE_SCURVE_ERROR_CODE;
		flag = true;
	};

	void set(DRV8880ErrCode err){
		err_code = (uint32_t)err;
		err_code_offset = err_code + OFFSET_DRV8880_ERROR_CODE;
		flag = true;
	};

	void set(DRV8880RegisterErrCode err){
		err_code = (uint32_t)err;
		err_code_offset = err_code + OFFSET_DRV8880_REGISTER_ERROR_CODE;
		flag = true;
	};


	void set(PwmPfcErrCode err) {
		err_code = (uint32_t)err;
		err_code_offset = err_code + OFFSET_PWM_PFC_ERROR_CODE;
		flag = true;
	}

	void set(DRV8880MasterProtocolErrCode err) {
		err_code = (uint32_t)err;
		err_code_offset = err_code + OFFSET_DRV8880_MASTER_PROTOCOL_ERROR_CODE;
		flag = true;
	}

	bool get(){
		return flag;
	};


	uint32_t getErrCode(){
		/* Reset flag to detect other errors */
		flag = false;
		return err_code_offset;
	};


	const char* getCodeString(uint32_t offset_err) {

		uint32_t err;

		if (offset_err >= OFFSET_TRAJECTORY_ERROR_CODE && offset_err < OFFSET_TRAJECTORY_ERROR_CODE + (uint32_t)TrajectoryErrCode::ALWAYS_AT_END) {
			err = offset_err - OFFSET_TRAJECTORY_ERROR_CODE;
			return getErrCodeString(TrajectoryErrCode(err));
		}
		else if (offset_err >= OFFSET_TRAJECTORY_REGISTER_ERROR_CODE && offset_err < OFFSET_TRAJECTORY_REGISTER_ERROR_CODE + (uint32_t)TrajectoryRegisterErrCode::ALWAYS_AT_END) {
			err = offset_err - OFFSET_TRAJECTORY_REGISTER_ERROR_CODE;
			return getErrCodeString(TrajectoryRegisterErrCode(err));
		}
		else if (offset_err >= OFFSET_PROFILE_CONCAVE_SCURVE_ERROR_CODE && offset_err < OFFSET_PROFILE_CONCAVE_SCURVE_ERROR_CODE + (uint32_t)ProfileConcaveScurveVelErrCode::ALWAYS_AT_END) {
			err = offset_err - OFFSET_PROFILE_CONCAVE_SCURVE_ERROR_CODE;
			return getErrCodeString(ProfileConcaveScurveVelErrCode(err));
		}
		else if (offset_err >= OFFSET_DRV8880_ERROR_CODE && offset_err < OFFSET_DRV8880_ERROR_CODE + (uint32_t)DRV8880ErrCode::ALWAYS_AT_END) {
			err = offset_err - OFFSET_DRV8880_ERROR_CODE;
			return getErrCodeString(DRV8880ErrCode(err));
		}
		else if (offset_err >= OFFSET_DRV8880_REGISTER_ERROR_CODE && offset_err < OFFSET_DRV8880_REGISTER_ERROR_CODE + (uint32_t)DRV8880RegisterErrCode::ALWAYS_AT_END) {
			err = offset_err - OFFSET_DRV8880_REGISTER_ERROR_CODE;
			return getErrCodeString(DRV8880RegisterErrCode(err));
		}
		else {
			return nullptr;
		}


	}

private:



	const char* getErrCodeString(DRV8880ErrCode err) {
	  switch (err) {
	  case DRV8880ErrCode::CURRENT_INVALID_PARAMETER: 						return "CURRENT_INVALID_PARAMETER";
	  case DRV8880ErrCode::MICROSTEP_INVALID_PARAMETER: 					return "MICROSTEP_INVALID_PARAMETER";
	  case DRV8880ErrCode::CURRENT_PINS_NOT_SET: 							return "CURRENT_PINS_NOT_SET";
	  case DRV8880ErrCode::MICROSTEP_PINS_NOT_SET: 							return "MICROSTEP_PINS_NOT_SET";
	  case DRV8880ErrCode::ALWAYS_AT_END: 									return "INTERNAL ERROR";
	  }
	  return nullptr;
	}

	const char* getErrCodeString(DRV8880RegisterErrCode err) {
	  switch (err) {
	  case DRV8880RegisterErrCode::DECODE_MRES_NOT_IMPLEMENTED: 			return "DECODE_MRES_NOT_IMPLEMENTED";
	  case DRV8880RegisterErrCode::ENCODE_MRES_NOT_IMPLEMENTED: 			return "ENCODE_MRES_NOT_IMPLEMENTED";
	  case DRV8880RegisterErrCode::DECODE_CURRENT_NOT_IMPLEMENTED: 			return "DECODE_CURRENT_NOT_IMPLEMENTED";
	  case DRV8880RegisterErrCode::ENCODE_CURRENT_NOT_IMPLEMENTED: 			return "ENCODE_CURRENT_NOT_IMPLEMENTED";
	  case DRV8880RegisterErrCode::DECODE_DIRECTION_NOT_IMPLEMENTED: 		return "DECODE_DIRECTION_NOT_IMPLEMENTED";
	  case DRV8880RegisterErrCode::ENCODE_DIRECTION_NOT_IMPLEMENTED: 		return "ENCODE_DIRECTION_NOT_IMPLEMENTED";

	  case DRV8880RegisterErrCode::ALWAYS_AT_END: 							return "INTERNAL ERROR";
	  }
	  return nullptr;
	}

	const char* getErrCodeString(TrajectoryErrCode err) {
	  switch (err) {
	  case TrajectoryErrCode::TRAJECTORY_NOT_IMPLEMENTED: 					return "TRAJECTORY_NOT_IMPLEMENTED";
	  case TrajectoryErrCode::TRAJECTORY_BUFFER_OVF: 						return "TRAJECTORY_BUFFER_OVF";
	  case TrajectoryErrCode::TRAJECTORY_PARAMS_OVF: 						return "TRAJECTORY_PARAMS_OVF";

	  case TrajectoryErrCode::ALWAYS_AT_END: 								return "INTERNAL ERROR";
	  }
	  return nullptr;
	}

	const char* getErrCodeString(TrajectoryRegisterErrCode err) {
	  switch (err) {
	  case TrajectoryRegisterErrCode::DECODE_PROFILE_NOT_IMPLEMENTED: 		return "DECODE_PROFILE_NOT_IMPLEMENTED";
	  case TrajectoryRegisterErrCode::ENCODE_PROFILE_NOT_IMPLEMENTED: 		return "ENCODE_PROFILE_NOT_IMPLEMENTED";

	  case TrajectoryRegisterErrCode::ALWAYS_AT_END: 						return "INTERNAL ERROR";
	  }
	  return nullptr;
	}

	const char* getErrCodeString(ProfileConcaveScurveVelErrCode err) {
	  switch (err) {
	  case ProfileConcaveScurveVelErrCode::PROFILE_CREATION_BAD_PARAMETERS: 	return "PROFILE_CREATION_BAD_PARAMETERS";
	  case ProfileConcaveScurveVelErrCode::PROFILE_CREATION_BUFFER_OVERFLOW: 	return "PROFILE_CREATION_BUFFER_OVERFLOW";
	  case ProfileConcaveScurveVelErrCode::BUFFER_REVERSE_MODE_WRONG_USE: 		return "BUFFER_REVERSE_MODE_WRONG_USE";
	  case ProfileConcaveScurveVelErrCode::BUFFER_MODE_NOT_IMPLEMENTED: 		return "BUFFER_MODE_NOT_IMPLEMENTED";
	  case ProfileConcaveScurveVelErrCode::BUFFER_WRONG_START_MODE: 			return "BUFFER_WRONG_START_MODE";
	  case ProfileConcaveScurveVelErrCode::PROFILE_NOT_CREATED: 				return "PROFILE_NOT_CREATED";

	  case ProfileConcaveScurveVelErrCode::ALWAYS_AT_END: 						return "INTERNAL ERROR";
	  }
	  return nullptr;
	}

	const char* getErrCodeString(PwmPfcErrCode err) {
	  switch (err) {
	  case PwmPfcErrCode::BUFFER_MODE_NOT_IMPLEMENTED: 							return "BUFFER_MODE_NOT_IMPLEMENTED";
	  case PwmPfcErrCode::BUFFER_REVERSE_MODE_WRONG_USE: 						return "BUFFER_REVERSE_MODE_WRONG_USE";
	  case PwmPfcErrCode::BUFFER_WRONG_START_MODE: 								return "BUFFER_WRONG_START_MODE";
	  case PwmPfcErrCode::INVALID_FREQUENCY_FOR_TOP_CALCULATION: 				return "INVALID_FREQUENCY_FOR_TOP_CALCULATION";

	  case PwmPfcErrCode::ALWAYS_AT_END: 										return "INTERNAL ERROR";
	  }
	  return nullptr;
	}







	uint32_t err_code = 65535;

};


extern ErrorHandle* p_err;

#endif /* ERROR_HANDLE_H_ */
