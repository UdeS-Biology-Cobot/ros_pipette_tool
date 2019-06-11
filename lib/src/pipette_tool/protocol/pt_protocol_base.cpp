/*
 * pt_protocol_base.cpp
 *
 *  Created on: Dec 5, 2018
 *      Author: biobot
 */


#include <robotic_tool/pipette_tool/protocol/pt_protocol_base.h>



void PtProtocolBase::crc8_atm(uint8_t* datagram, uint8_t datagramLength)
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









bool PtProtocolBase::err_get(PtProtocolErr* p_err)
{
	if (*p_err == PtProtocolErr::NONE) {
		return false;
	}
	return true;
}

void PtProtocolBase::err_clear(PtProtocolErr* p_err)
{
	*p_err = PtProtocolErr::NONE;
}
