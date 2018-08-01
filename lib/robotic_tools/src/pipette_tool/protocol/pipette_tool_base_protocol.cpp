/*
 * drv8880_protocol.cpp
 *
 *  Created on: May 30, 2018
 *      Author: robotic_tools
 */





#include <robotic_tools/error_handle/error_handle.h>
#include <robotic_tools/pipette_tool/protocol/pipette_tool_base_protocol.h>


/*Fuction to get gcd of a and b*/
int PipetteToolBaseProtocol::gcd(int a,int b)
{
    if(b == 0)
    return a;

    else
    return gcd(b, a%b);
}


/*Function to left rotate arr[] of siz n by d*/
void PipetteToolBaseProtocol::leftRotate(uint8_t arr[], int d, int n)
{
	for (int i = 0; i < gcd(d, n); i++)
	{
		/* move i-th values of blocks */
		int temp = arr[i];
		int j = i;

		while(1)
		{
			int k = j + d;
			if (k >= n)
				k = k - n;

			if (k == i)
				break;

			arr[j] = arr[k];
			j = k;
		}
		arr[j] = temp;
	}
}







PipetteToolBaseProtocol::PipetteToolBaseProtocol(uint8_t slave_addr)
:SLAVE_FRAME(slave_addr)
{}

PipetteToolBaseProtocol::~PipetteToolBaseProtocol()
{}


void PipetteToolBaseProtocol::crc8_atm(uint8_t* datagram, uint8_t datagramLength)
{
	short i,j;
	uint8_t* crc = datagram + (datagramLength-1); // CRC located in last byte of message
	uint8_t currentByte;
	*crc = 0;
	for (i=0; i<(datagramLength-1); i++) {			// Execute for all bytes of a message
		currentByte = datagram[i];					// Retrieve a byte to be sent from Array
		for (j=0; j<8; j++) {
			if ((*crc >> 7) ^ (currentByte&0x01))	// Update CRC based result of XOR operation
			{
				*crc = (*crc << 1) ^ 0x07;
			}
			else
			{
				*crc = (*crc << 1);
			}
			currentByte = currentByte >> 1;
		} // for CRC bit
	} // for message byte
}


bool PipetteToolBaseProtocol::read_datagram(uint8_t data) {

	uint8_t sync_frame;
	uint8_t slave_frame;
	uint8_t crc_frame;

	if (DATAGRAM_FOUND) {
		DATAGRAM_FOUND = false;

		for(int i = 0; i < DATAGRAM_SIZE; i++) {
			datagram[i] = 0;
		}
	}

	leftRotate(datagram, 1, DATAGRAM_SIZE);
	datagram[DATAGRAM_SIZE - 1] = data;

	sync_frame = datagram[SYNC_IDX];

	if (sync_frame == SYNC_FRAME) {
		/* Check for slave adress	*/
		slave_frame = datagram[SLAVE_IDX];

		if (slave_frame == SLAVE_FRAME) {

			crc_frame = datagram[CRC_IDX];
			crc8_atm(datagram, DATAGRAM_SIZE);

			if (crc_frame == datagram[CRC_IDX]) {
				DATAGRAM_FOUND = true;
			}
		}
	}

	return DATAGRAM_FOUND;
}



void PipetteToolBaseProtocol::setSyncFrame()
{
	datagram[SYNC_IDX] = SYNC_FRAME;
}

void PipetteToolBaseProtocol::setSlaveFrame()
{
	datagram[SLAVE_IDX] = SLAVE_FRAME;
}




void PipetteToolBaseProtocol::setDataFrame(uint32_t data)
{
	//data = htonl(data);

	datagram[DATA0_IDX] = (data & 0xFF000000) >> 24;
	datagram[DATA1_IDX] = (data & 0x00FF0000) >> 16;
	datagram[DATA2_IDX] = (data & 0x0000FF00) >> 8;
	datagram[DATA3_IDX] = (data & 0x000000FF) >> 0;
}




uint32_t PipetteToolBaseProtocol::unpackDataFrame()
{
	uint32_t data = 0;

	data |= (uint32_t)datagram[DATA0_IDX] << 24;
	data |= (uint32_t)datagram[DATA1_IDX] << 16;
	data |= (uint32_t)datagram[DATA2_IDX] << 8;
	data |= (uint32_t)datagram[DATA3_IDX] << 0;

	return data;
}


PipetteToolBaseProtocol::RegisterRW PipetteToolBaseProtocol::decode_register_rw(uint32_t data)
{
	uint8_t b_data = (data & REGISTER_RW_BM) >> REGISTER_RW_BP;

	RegisterRW reg_rw = RegisterRW(b_data);

	return reg_rw;
}

uint32_t PipetteToolBaseProtocol::encode_register_rw(RegisterRW reg_rw)
{
	return ((uint32_t)reg_rw) << REGISTER_RW_BP;
}




PipetteToolBaseProtocol::RegisterType PipetteToolBaseProtocol::get_register_type(uint8_t offset_reg)
{

	if (offset_reg >= OFFSET_REG_PIPETTE_TOOL && offset_reg < (OFFSET_REG_PIPETTE_TOOL + SIZE_REG_PIPETTE_TOOL))
	{
		return RegisterType::PIPETTE_TOOL;
	}
	else if (offset_reg >= OFFSET_REG_STEPPER_MOTOR && offset_reg < (OFFSET_REG_STEPPER_MOTOR + SIZE_REG_STEPPER_MOTOR))
	{
		return RegisterType::STEPPER_MOTOR;
	}
	else if (offset_reg >= OFFSET_REG_STEPPER_DRIVER && offset_reg < (OFFSET_REG_STEPPER_DRIVER + SIZE_REG_STEPPER_DRIVER))
	{
		return RegisterType::STEPPER_DRIVER;
	}
	else if (offset_reg >= OFFSET_REG_TRAJECTORY && offset_reg < (OFFSET_REG_TRAJECTORY + SIZE_REG_TRAJECTORY))
	{
		return RegisterType::TRAJECTORY;
	}
	else if (offset_reg >= OFFSET_REG_LEADSCREW && offset_reg < (OFFSET_REG_LEADSCREW + SIZE_REG_LEADSCREW))
	{
		return RegisterType::LEADSCREW;
	}
	else if (offset_reg >= OFFSET_REG_ERROR && offset_reg < (OFFSET_REG_ERROR + SIZE_REG_ERROR))
	{
		return RegisterType::ERROR;
	}
	else {
		/* TODO change error */
		p_err->set(PwmPfcErrCode::INVALID_FREQUENCY_FOR_TOP_CALCULATION);
	}

	return RegisterType::ERROR;
}




uint8_t PipetteToolBaseProtocol::get_register(uint8_t offset_reg)
{
	RegisterType reg_type = get_register_type(offset_reg);


	switch (reg_type) {
		case RegisterType::PIPETTE_TOOL:
			return offset_reg - OFFSET_REG_PIPETTE_TOOL;
		case RegisterType::STEPPER_MOTOR:
			return offset_reg - OFFSET_REG_STEPPER_MOTOR;
		case RegisterType::STEPPER_DRIVER:
			return offset_reg - OFFSET_REG_STEPPER_DRIVER;
		case RegisterType::TRAJECTORY:
			return offset_reg - OFFSET_REG_TRAJECTORY;
		case RegisterType::LEADSCREW:
			return offset_reg - OFFSET_REG_LEADSCREW;
		case RegisterType::ERROR:
			return offset_reg - OFFSET_REG_ERROR;
		default:
			/* TODO set error */
			return 0;
	}
}

uint8_t PipetteToolBaseProtocol::get_register_offset(RegisterType reg_type, uint8_t reg)
{

	switch (reg_type) {
		case RegisterType::PIPETTE_TOOL:
			return reg + OFFSET_REG_PIPETTE_TOOL;
		case RegisterType::STEPPER_MOTOR:
			return reg + OFFSET_REG_STEPPER_MOTOR;
		case RegisterType::STEPPER_DRIVER:
			return reg + OFFSET_REG_STEPPER_DRIVER;
		case RegisterType::TRAJECTORY:
			return reg + OFFSET_REG_TRAJECTORY;
		case RegisterType::LEADSCREW:
			return reg + OFFSET_REG_LEADSCREW;
		case RegisterType::ERROR:
			return reg + OFFSET_REG_ERROR;
		default:
			/* TODO set error */
			return 0;
	}
}


