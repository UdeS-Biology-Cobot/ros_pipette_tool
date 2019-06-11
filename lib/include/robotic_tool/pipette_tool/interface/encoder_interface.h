/*
 * encoder_interface.h
 *
 *  Created on: Nov 30, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_ENCODER_INTERFACE_ENCODER_INTERFACE_H_
#define LIB_BIOBOT_BIOBOT_ENCODER_INTERFACE_ENCODER_INTERFACE_H_

#include <stdint.h>

// Error Code for all encoders
enum class EncoderInterfaceErr : uint32_t {
	NONE,

	TIMEOUT,
	INTERNAL_ERROR,
};



//class AMT21Abstract


class EncoderInterface
{
public:
	//EncoderInterface();
	virtual ~EncoderInterface() {};

	virtual uint32_t get_resolution() = 0;
	virtual  int32_t read(EncoderInterfaceErr* p_err) = 0;
	virtual  int32_t read(int32_t offset, EncoderInterfaceErr* p_err) = 0;
	virtual     void reset(EncoderInterfaceErr* p_err) = 0;




	virtual bool err_get(EncoderInterfaceErr* p_err) = 0;
	virtual void err_clear(EncoderInterfaceErr* p_err) = 0;
};










#endif /* LIB_BIOBOT_BIOBOT_ENCODER_INTERFACE_ENCODER_INTERFACE_H_ */
