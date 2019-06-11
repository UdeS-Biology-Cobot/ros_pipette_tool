/*
 * amt21_interface.h
 *
 *  Created on: Nov 30, 2018
 *      Author: biobot
 */

#ifndef LIB_BIOBOT_BIOBOT_ENCODER_INTERFACE_AMT21_INTERFACE_H_
#define LIB_BIOBOT_BIOBOT_ENCODER_INTERFACE_AMT21_INTERFACE_H_



//#if defined(__linux__)
//#include <robotic_tool/encoder/posix/amt21.h>
//#elif defined(ARDUINO)
#if defined(ARDUINO)
#include <robotic_tool/encoder/arduino/amt21.h>


#include <robotic_tool/pipette_tool/interface/encoder_interface.h>

class AMT21Interface : public EncoderInterface, public AMT21
{
public:
	AMT21Interface(UartSerial* port, uint32_t baudrate, Rotation rot, EncoderInterfaceErr* p_err);

	uint32_t get_resolution() override;
	 int32_t read(EncoderInterfaceErr* p_err) override;
	 int32_t read(int32_t offset, EncoderInterfaceErr* p_err) override;
	    void reset(EncoderInterfaceErr* p_err) override;


		bool err_get(EncoderInterfaceErr* p_err) override;
		void err_clear(EncoderInterfaceErr* p_err) override;

protected:
	    EncoderInterfaceErr modify_err_code(AMT21Err* p_err);
};


#endif





#endif /* LIB_BIOBOT_BIOBOT_ENCODER_INTERFACE_AMT21_INTERFACE_H_ */
