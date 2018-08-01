/*
 * drv8880_protocol.h
 *
 *  Created on: May 30, 2018
 *      Author: robotic_tools
 */

#ifndef PIPETTE_TOOL_BASE_PROTOCOL_H_
#define PIPETTE_TOOL_BASE_PROTOCOL_H_

/*
#include <robotic_tools/stepper_driver/register/stepper_driver_register.h>
#include <robotic_tools/error_handle/register/error_handle_register.h>
#include <robotic_tools/pipette_tool/register/pipette_tool_base_register.h>
#include <robotic_tools/trajectory/register/trajectory_base_register.h>
*/
//#include <robotic_tools/stepper_motor/register/stepper_motor_master_register.h>
//#include <robotic_tools/leadscrew/register/leadscrew_master_register.h>


#include <robotic_tools/pipette_tool/register/pipette_tool_base_register.h>
#include <robotic_tools/stepper_motor/register/stepper_motor_base_register.h>
#include <robotic_tools/stepper_driver/register/stepper_driver_register.h>
#include <robotic_tools/trajectory/register/trajectory_base_register.h>
#include <robotic_tools/leadscrew/register/leadscrew_base_register.h>
#include <robotic_tools/error_handle/register/error_handle_register.h>


#include <stdint.h>

class PipetteToolBaseProtocol
{


public:

	PipetteToolBaseProtocol(uint8_t slave_addr);
	virtual ~PipetteToolBaseProtocol();

	void crc8_atm(uint8_t* datagram, uint8_t datagramLength);

	//byte* read_response(byte data);		/* Used by master */

private:
	const uint8_t SYNC_FRAME	= 0xAA;
	const uint8_t SLAVE_FRAME;

protected:
	static const uint8_t SIZE_REG_PIPETTE_TOOL			= (uint8_t)PipetteToolBaseRegister::Registers::SIZE;
	static const uint8_t SIZE_REG_STEPPER_MOTOR			= (uint8_t)StepperMotorBaseRegister::Registers::SIZE;
	static const uint8_t SIZE_REG_STEPPER_DRIVER		= (uint8_t)StepperDriverRegister::Registers::SIZE;
	static const uint8_t SIZE_REG_TRAJECTORY			= (uint8_t)TrajectoryBaseRegister::Registers::SIZE;
	static const uint8_t SIZE_REG_LEADSCREW				= (uint8_t)LeadscrewBaseRegister::Registers::SIZE;
	static const uint8_t SIZE_REG_ERROR					= (uint8_t)ErrorHandleRegister::Registers::SIZE;

	static const uint8_t OFFSET_REG_PIPETTE_TOOL 		= 0x00;
	static const uint8_t OFFSET_REG_STEPPER_MOTOR		= OFFSET_REG_PIPETTE_TOOL 	+ SIZE_REG_PIPETTE_TOOL;
	static const uint8_t OFFSET_REG_STEPPER_DRIVER		= OFFSET_REG_STEPPER_MOTOR 	+ SIZE_REG_STEPPER_MOTOR;
	static const uint8_t OFFSET_REG_TRAJECTORY			= OFFSET_REG_STEPPER_DRIVER + SIZE_REG_STEPPER_DRIVER;
	static const uint8_t OFFSET_REG_LEADSCREW			= OFFSET_REG_TRAJECTORY 	+ SIZE_REG_TRAJECTORY;
	static const uint8_t OFFSET_REG_ERROR				= OFFSET_REG_LEADSCREW 		+ SIZE_REG_LEADSCREW;



	enum class RegisterType : uint8_t
	{
		PIPETTE_TOOL,
		STEPPER_MOTOR,
		STEPPER_DRIVER,
		TRAJECTORY,
		LEADSCREW,
		ERROR,
	};


	RegisterType get_register_type(uint8_t offset_reg);
	uint8_t get_register(uint8_t offset_reg);
	uint8_t get_register_offset(RegisterType reg_type, uint8_t reg);

/////////////////////////////////////////
public:
	static const uint8_t DATAGRAM_SIZE	= 8;
	static const uint8_t MAX_REGISTERS	= 128;

	enum class RegisterRW : bool
	{
		REGISTER_READ = false,
		REGISTER_WRITE = true,
	};


/*
	enum class ProtocolReg : uint8_t
	{
		DRIVER,
		TRAJECTORY,
		PIPETTE_TOOL,
		ERROR,
	};
*/


private:
	bool DATAGRAM_FOUND = false;



protected:
	bool read_datagram(uint8_t data);
	int gcd(int a,int b);
	void leftRotate(uint8_t arr[], int d, int n);

	void setDataFrame(uint32_t data);
	void setSyncFrame();
	void setSlaveFrame();



	uint32_t unpackDataFrame();


	uint8_t datagram[DATAGRAM_SIZE];


	//Trajectory::Profile profile = Trajectory::Profile::NONE;



	enum DatagramFrames {
		SYNC_IDX,
		SLAVE_IDX,
		REGISTER_IDX,
		DATA0_IDX,
		DATA1_IDX,
		DATA2_IDX,
		DATA3_IDX,
		CRC_IDX
	};









	/* REGISTER FRAME */
	static const uint8_t REGISTER_RW_BP 		=  7;
	static const uint8_t REGISTER_BP 			=  0;

	static const uint8_t REGISTER_RW_BM 		=  0x80;
	static const uint8_t REGISTER_BM 			=  0x7F;

	/**/



	//static const bool REGISTER_READ 			=  false;
	//static const bool REGISTER_WRITE 			=  true;


	RegisterRW decode_register_rw(uint32_t data);

	uint32_t encode_register_rw(RegisterRW);







};









#endif /* PIPETTE_TOOL_BASE_PROTOCOL_H_ */
