/*
 * tmc2208_base.cpp
 *
 *  Created on: Nov 30, 2018
 *      Author: biobot
 */


#include <robotic_tool/driver/tmc2208_base.h>
#include <math.h>


// TODO remove
#if defined(ARDUINO)
#include <Arduino.h>
#endif

TMC2208Base::TMC2208Base(uint32_t baudrate)
: SERIAL_BEFORE_READ_DELAY(get_serial_rx_delay(baudrate))
{





	/*
	if (err_get(p_err)) {return;}

	init_pin();
	enable_motor();

	port->begin(baudrate);
	delayMicroseconds(500);	// Corresponds to low baudrate + RC filter (5 Tau -> 1k_ohm + Capacitor)
*/
	// Fill all registers data

}









/*
void TMC2208Base::pin_init()
{
	*mode_half_duplex |= pin_half_duplex;
	*out_half_duplex |= pin_half_duplex;	// 1 = TX enable, 0 = RX enable

	*mode_dir |= pin_dir;
	*out_dir &= ~pin_dir;	//

	*mode_en |= pin_en;
	*out_en |= pin_en;	// 0 = enable, 1 = disable
}
*/

void TMC2208Base::enable_motor()
{
	//*out_en &= ~pin_en;
	set_enable_pin(LogicLevel::LOW_LEVEL);
}

void TMC2208Base::disable_motor()
{
	//*out_en |= pin_en;
	set_enable_pin(LogicLevel::HIGH_LEVEL);
}

void TMC2208Base::set_direction(RotationDir dir, TMC2208Err* p_err)
{
	// If already in the right direction
	if (dir == direction) {
		return;
	}

	if (dir == RotationDir::CW) {
		set_direction_pin(LogicLevel::LOW_LEVEL);
		//*out_dir &= ~pin_dir;
	}
	else if (dir == RotationDir::CCW) {
		//*out_dir |= pin_dir;
		set_direction_pin(LogicLevel::HIGH_LEVEL);
	}
	else {
		*p_err = TMC2208Err::INVALID_DIRECTION;
		return;
	}

	direction = dir;
}

TMC2208Base::MRES TMC2208Base::get_max_mres() {
	return MRES::TWO_HUNDRED_FIFTY_SIXTH_STEP;
}


void TMC2208Base::crc8_atm(uint8_t* datagram, uint8_t len)
{
	short i,j;
	uint8_t* crc = datagram + (len-1); // CRC located in last byte of message
	uint8_t current_byte;
	*crc = 0;
	for (i=0; i< len - 1; i++) {			// Execute for all bytes of a message
		current_byte = datagram[i];					// Retrieve a byte to be sent from Array
		for (j=0; j<8; j++) {
			if ((*crc >> 7) ^ (current_byte & 0x01))	// Update CRC based result of XOR operation
			{
				*crc = (*crc << 1) ^ 0x07;
			}
			else
			{
				*crc = (*crc << 1);
			}
			current_byte = current_byte >> 1;
		} // for CRC bit
	} // for message byte
}


void TMC2208Base::set_pdn_disable(PDN_DISABLE pdn_disable, TMC2208Err* p_err)
{
	uint32_t data = encode_pdn_disable(pdn_disable);
	write_register(&GCONF, data, p_err);
}

TMC2208Base::PDN_DISABLE TMC2208Base::get_pdn_disable(TMC2208Err* p_err)
{
	uint32_t data = read_register(&GCONF, p_err);
	return decode_pdn_disable(data);
}


void TMC2208Base::set_en_spread_cycle(EN_SPREAD_CYCLE en_spread_cycle, TMC2208Err* p_err)
{
	uint32_t data = encode_en_spread_cycle(en_spread_cycle);
	write_register(&GCONF, data, p_err);
}

TMC2208Base::EN_SPREAD_CYCLE TMC2208Base::get_en_spread_cycle(TMC2208Err* p_err)
{
	uint32_t data = read_register(&GCONF, p_err);
	return decode_en_spread_cycle(data);
}


void TMC2208Base::set_toff(uint8_t toff, TMC2208Err* p_err)
{
	uint32_t data = encode_toff(toff);
	write_register(&CHOPCONF, data, p_err);
}

uint8_t TMC2208Base::get_toff(TMC2208Err* p_err)
{
	uint32_t data = read_register(&CHOPCONF, p_err);
	return decode_toff(data);
}




void TMC2208Base::set_mstep_reg_select(MSTEP_REG_SELECT mstep_reg_select, TMC2208Err* p_err)
{
	uint32_t data = encode_mstep_reg_select(mstep_reg_select);
	write_register(&GCONF, data, p_err);
	if(err_get(p_err)) {return;}

	read_register(&CHOPCONF, p_err);	// Reread CHOPCONF because a write to MSTEP_REG_SELECT::REGISTER will change CHOPCONF
}

TMC2208Base::MSTEP_REG_SELECT TMC2208Base::get_mstep_reg_select(TMC2208Err* p_err)
{
	uint32_t data = read_register(&GCONF, p_err);
	return decode_mstep_reg_select(data);
}





void TMC2208Base::set_ihold(uint8_t ihold, TMC2208Err* p_err)
{
	uint32_t data = encode_ihold(ihold);
	write_register(&IHOLD_IRUN, data, p_err);
}

uint8_t TMC2208Base::get_ihold(TMC2208Err* p_err)
{
	uint32_t data = read_register(&IHOLD_IRUN, p_err);
	return decode_ihold(data);
}

void TMC2208Base::set_irun(uint8_t irun, TMC2208Err* p_err)
{
	uint32_t data = encode_irun(irun);
	write_register(&IHOLD_IRUN, data, p_err);
}

uint8_t TMC2208Base::get_irun(TMC2208Err* p_err)
{
	uint32_t data = read_register(&IHOLD_IRUN, p_err);
	return decode_irun(data);
}

void TMC2208Base::set_iholddelay(uint8_t iholddelay, TMC2208Err* p_err)
{
	uint32_t data = encode_iholddelay(iholddelay);
	write_register(&IHOLD_IRUN, data, p_err);
}

uint8_t TMC2208Base::get_iholddelay(TMC2208Err* p_err)
{
	uint32_t data = read_register(&IHOLD_IRUN, p_err);
	return decode_iholddelay(data);
}

void TMC2208Base::set_mres(MRES mres, TMC2208Err* p_err)
{
	uint32_t data = encode_mres(mres);
	write_register(&CHOPCONF, data, p_err);
}


void TMC2208Base::set_microstep(uint32_t microstep, TMC2208Err* p_err)
{
	MRES mres = encode_microstep(microstep, p_err);
	if (err_get(p_err)) {return;}

	set_mres(mres, p_err);
}


uint32_t TMC2208Base::get_microstep(TMC2208Err* p_err) {
	MRES mres = get_mres(p_err);
	if (err_get(p_err)) {return 0 ;}

	uint32_t microstep = decode_mres(mres, p_err);
	return microstep;
}


uint32_t TMC2208Base::get_max_microstep(TMC2208Err* p_err) {
	MRES mres = get_max_mres();
	uint32_t microstep = decode_mres(mres, p_err);
	return microstep;
}


TMC2208Base::MRES TMC2208Base::encode_microstep(uint32_t microstep, TMC2208Err* p_err)
{

	switch (microstep) {

	case 1:
		return MRES::FULL_STEP;
	case 2:
		return MRES::HALF_STEP;
	case 4:
		return MRES::QUARTER_STEP;
	case 8:
		return MRES::EIGHTH_STEP;
	case 16:
		return MRES::SIXTEENTH_STEP;
	case 32:
		return MRES::THIRTY_SECOND_STEP;
	case 64:
		return MRES::SIXTY_FOURTH_STEP;
	case 128:
		return MRES::ONE_HUNDRED_TWENTY_EIGHT_STEP;
	case 256:
		return MRES::TWO_HUNDRED_FIFTY_SIXTH_STEP;
	}

	*p_err = TMC2208Err::INVALID_MRES;
	return MRES::FULL_STEP;
}


TMC2208Base::MRES TMC2208Base::get_mres(TMC2208Err* p_err)
{
	uint32_t data = read_register(&CHOPCONF, p_err);
	return decode_microstep(data);
}


void TMC2208Base::set_intpol(INTPOL intpol, TMC2208Err* p_err)
{
	uint32_t data = encode_intpol(intpol);
	write_register(&CHOPCONF, data, p_err);
}

TMC2208Base::INTPOL TMC2208Base::get_intpol(TMC2208Err* p_err)
{
	uint32_t data = read_register(&CHOPCONF, p_err);
	return decode_intpol(data);
}






void TMC2208Base::read_all_registers(TMC2208Err* p_err)
{
	read_register(&GCONF, p_err);
	if (err_get(p_err)) {return;}

	read_register(&IFCNT, p_err);
	if (err_get(p_err)) {return;}

	// IHOLD_IRUN is WRITE ONLY

	read_register(&CHOPCONF, p_err);
	if (err_get(p_err)) {return;}


	uint32_t gstat = read_register(&GSTAT, p_err);
	if (err_get(p_err)) {return;}

	uint32_t diagnostic = read_register(&DRV_STATUS, p_err);
	if (err_get(p_err)) {return;}
}


uint32_t TMC2208Base::get_serial_rx_delay(uint32_t baudrate)
{
	float READ_DATAFRAME_TIME = (1.0/float(baudrate))*(10.0 * (float)READ_ACCESS_REPLY_BUF_LEN);
	float REPLY_DELAY = ((1.0/float(baudrate))*(float)SENDDELAY);
	float ADDITIONNAL_DELAY  = 0.002;

	return static_cast<uint32_t>(roundf((REPLY_DELAY + READ_DATAFRAME_TIME + ADDITIONNAL_DELAY)*1000.0));

}


// Get register data
uint32_t TMC2208Base::read_register(Register* p_reg, TMC2208Err* p_err) {

	uint8_t crc;
	uint32_t retry_ctr;

	TMC2208Err local_ec;



	for (retry_ctr = 0; retry_ctr < MAX_RETRY; retry_ctr++) {
		local_ec = TMC2208Err::NONE;
		//*out_half_duplex |= pin_half_duplex;
		set_halfduplex_pin(LogicLevel::HIGH_LEVEL);
		// Set Register
		tx_buf[REGISTER_IDX] = p_reg->addr;

		// Calculate CRC
		crc8_atm(tx_buf, READ_ACCESS_REQUEST_BUF_LEN);

		// Flush serial rx buffer
		serial_flush_rx();
		/*
		while (port->available() > 0){
			port->read();
		};
*/


		// Write buffer to serial
		serial_write(tx_buf, READ_ACCESS_REQUEST_BUF_LEN);
		/*
		byte_ctr = 0;
		while (byte_ctr != READ_ACCESS_REQUEST_BUF_LEN) {
			byte_ctr += port->write(tx_buf, READ_ACCESS_REQUEST_BUF_LEN);
		}
*/
		// Wait till last stop bit of buffer have been transmitted
		//while(!sercom->USART.INTFLAG.bit.TXC);
		serial_flush_tx();

		//*out_half_duplex &= ~pin_half_duplex;
		set_halfduplex_pin(LogicLevel::LOW_LEVEL);
		// Delay
		delay_ms(SERIAL_BEFORE_READ_DELAY);

		serial_read(rx_buf, READ_ACCESS_REPLY_BUF_LEN, &local_ec);


		// Validate crc
		if (local_ec == TMC2208Err::NONE) {
			crc = rx_buf[CRC_IDX];
			crc8_atm(rx_buf, READ_ACCESS_REPLY_BUF_LEN);

			if (crc != rx_buf[CRC_IDX]) {
				local_ec = TMC2208Err::CRC_MISMATCH;
			}
		}

		// Validate master address
		if (local_ec == TMC2208Err::NONE) {
			if (rx_buf[ADDRESS_IDX] != MASTER_ADDRESS) {
				local_ec = TMC2208Err::MASTER_ADDRESS_MISMATCH;
			}
		}






		if (local_ec == TMC2208Err::NONE) {
			break;
		}
	}

	//*out_half_duplex |= pin_half_duplex;
	set_halfduplex_pin(LogicLevel::HIGH_LEVEL);

	if (retry_ctr == MAX_RETRY) {
		*p_err = TMC2208Err::MAX_RETRY_REACHED;
		return 0;
	}

	// Copy Register value
	uint32_t data = decode_dataframe(rx_buf);
	p_reg->data = data;

	return data;
}



// Get register data
void TMC2208Base::write_register(Register* p_reg, uint32_t data, TMC2208Err* p_err) {

	uint8_t if_counter = IFCNT.data;	// For silencing warning
	uint32_t retry_ctr;
	TMC2208Err local_ec;

	for (retry_ctr = 0; retry_ctr < MAX_RETRY; retry_ctr++) {
		local_ec = TMC2208Err::NONE;
		//*out_half_duplex |= pin_half_duplex;
		set_halfduplex_pin(LogicLevel::HIGH_LEVEL);

		if (p_reg->rw_field == RwField::WRITE_ONLY) {
			if_counter = read_register(&IFCNT, p_err);
			if (err_get(p_err)) {return;}
		}

		// Set Register + RW bit
		tx_buf[REGISTER_IDX] = p_reg->addr;
		tx_buf[REGISTER_IDX] |= RW_WRITE;

		encode_dataframe(tx_buf, data);

		// Calculate CRC
		crc8_atm(tx_buf, WRITE_ACCESS_BUF_LEN);

		// Flush serial rx buffer
		serial_flush_rx();
		/*
		while (port->available() > 0){
			port->read();
		};
*/
		// Write buffer to serial
		serial_write(tx_buf, WRITE_ACCESS_BUF_LEN);
		/*
		byte_ctr = 0;
		while (byte_ctr != WRITE_ACCESS_BUF_LEN) {
			byte_ctr += port->write(tx_buf, WRITE_ACCESS_BUF_LEN);
		}
		*/

		//while(!sercom->USART.INTFLAG.bit.TXC);
		serial_flush_tx();


		switch (p_reg->rw_field) {
			case RwField::READ_WRITE:
				rw_read_write(p_reg, tx_buf, &local_ec);
				break;
			case RwField::WRITE_ONLY:
				rw_write_only(p_reg, tx_buf, if_counter, &local_ec);
				break;
			case RwField::READ_ONLY:
				local_ec = TMC2208Err::NOT_IMPLEMENTED;
				break;

			case RwField::READ_WRITE_CLEAR:
				rw_read_write_clear(p_reg, tx_buf, &local_ec);
				break;

		}

		if (local_ec == TMC2208Err::NONE) {
			break;
		}
		//ec = TMC2208::ErrorCode::NONE;

	}

	//*out_half_duplex |= pin_half_duplex;
	set_halfduplex_pin(LogicLevel::HIGH_LEVEL);


	if (retry_ctr == MAX_RETRY) {
		*p_err = TMC2208Err::MAX_RETRY_REACHED;
	}
}



void TMC2208Base::rw_read_write(Register* p_reg, uint8_t* buf, TMC2208Err* p_err)
{
	// Compare transmited and received register value
	uint32_t tx_datareg = decode_dataframe(buf);
	uint32_t rx_datareg = read_register(p_reg, p_err);
	if (err_get(p_err)) {return;}

	if (tx_datareg != rx_datareg) {
		*p_err = TMC2208Err::REGISTER_UNCHANGED;
	}
}

void TMC2208Base::rw_write_only(Register* p_reg, uint8_t* buf, uint8_t prev_if_counter, TMC2208Err* p_err)
{
	uint32_t tx_datareg = decode_dataframe(buf);

	uint32_t curr_if_counter = read_register(&IFCNT, p_err);
	if (err_get(p_err)) {return;}

	if (prev_if_counter + 1 == (uint8_t)curr_if_counter) {
		p_reg->data = tx_datareg;
	}
	else {
		*p_err = TMC2208Err::IFCNT_UNCHANGED;
	}
}


void TMC2208Base::rw_read_write_clear(Register* p_reg, uint8_t* buf, TMC2208Err* p_err)
{
	// Compare transmited and received register value
	uint32_t tx_datareg = decode_dataframe(buf);
	uint32_t rx_datareg = read_register(p_reg, p_err);
	if (err_get(p_err)) {return;}

	// Clear on 1
	if ((tx_datareg & rx_datareg) != 0) {
		*p_err = TMC2208Err::REGISTER_NOT_CLEARED;
	}
}



TMC2208Base::MRES TMC2208Base::decode_microstep(uint32_t data)
{
	uint8_t _mres = (data & MRES_BM) >> MRES_BP;
	MRES mres = MRES(_mres);
	return mres;
}

uint32_t TMC2208Base::decode_mres(MRES mres, TMC2208Err* p_err)
{
	switch (mres) {
		case MRES::FULL_STEP:
			return 1;
		case MRES::HALF_STEP:
			return 2;
		case MRES::QUARTER_STEP:
			return 4;
		case MRES::EIGHTH_STEP:
			return 8;
		case MRES::SIXTEENTH_STEP:
			return 16;
		case MRES::THIRTY_SECOND_STEP:
			return 32;
		case MRES::SIXTY_FOURTH_STEP:
			return 64;
		case MRES::ONE_HUNDRED_TWENTY_EIGHT_STEP:
			return 128;
		case MRES::TWO_HUNDRED_FIFTY_SIXTH_STEP:
			return 256;
	}


	*p_err = TMC2208Err::INVALID_MRES;
	return 0;
}




uint32_t TMC2208Base::encode_mres(MRES mres)
{
	// Only write change to mres bits register
	uint32_t data = (CHOPCONF.data & ~MRES_BM);
	uint32_t _mres = (uint32_t)mres;

	data |= (_mres << MRES_BP);

	return data;
}


TMC2208Base::INTPOL TMC2208Base::decode_intpol(uint32_t data)
{
	uint8_t _intpol = (data & INTPOL_BM) >> INTPOL_BP;
	INTPOL intpol = INTPOL(_intpol);
	return intpol;
}

uint32_t TMC2208Base::encode_intpol(INTPOL intpol)
{
	// Only write change to mres bits register
	uint32_t data = (CHOPCONF.data & ~INTPOL_BM);
	uint32_t _intpol = (uint32_t)intpol;

	data |= (_intpol << INTPOL_BP);

	return data;
}


TMC2208Base::MSTEP_REG_SELECT TMC2208Base::decode_mstep_reg_select(uint32_t data)
{
	uint8_t _mstep_reg_select = (data & MSTEP_REG_SELECT_BM) >> MSTEP_REG_SELECT_BP;
	MSTEP_REG_SELECT mstep_reg_select = MSTEP_REG_SELECT(_mstep_reg_select);
	return mstep_reg_select;
}

uint32_t TMC2208Base::encode_mstep_reg_select(MSTEP_REG_SELECT mstep_reg_select)
{
	uint32_t data = (GCONF.data & ~MSTEP_REG_SELECT_BM);
	uint32_t _mstep_reg_select = (uint32_t)mstep_reg_select;

	data |= (_mstep_reg_select << MSTEP_REG_SELECT_BP);

	return data;
}


TMC2208Base::PDN_DISABLE TMC2208Base::decode_pdn_disable(uint32_t data)
{
	uint8_t _pdn_disable = (data & PDN_DISABLE_BM) >> PDN_DISABLE_BP;
	PDN_DISABLE pdn_disable = PDN_DISABLE(_pdn_disable);
	return pdn_disable;
}

uint32_t TMC2208Base::encode_pdn_disable(PDN_DISABLE pdn_disable)
{
	uint32_t data = (GCONF.data & ~PDN_DISABLE_BM);
	uint32_t _pdn_disable = (uint32_t)pdn_disable;

	data |= (_pdn_disable << PDN_DISABLE_BP);

	return data;
}



TMC2208Base::EN_SPREAD_CYCLE TMC2208Base::decode_en_spread_cycle(uint32_t data)
{
	uint8_t _en_spread_cycle = (data & EN_SPREAD_CYCLE_BM) >> EN_SPREAD_CYCLE_BP;
	EN_SPREAD_CYCLE en_spread_cycle = EN_SPREAD_CYCLE(_en_spread_cycle);
	return en_spread_cycle;
}

uint32_t TMC2208Base::encode_en_spread_cycle(EN_SPREAD_CYCLE en_spread_cycle)
{
	uint32_t data = (GCONF.data & ~EN_SPREAD_CYCLE_BM);
	uint32_t _en_spread_cycle = (uint32_t)en_spread_cycle;

	data |= (_en_spread_cycle << EN_SPREAD_CYCLE_BP);

	return data;
}



uint8_t TMC2208Base::decode_toff(uint32_t data)
{
	uint8_t toff = (data & TOFF_BM) >> TOFF_BP;
	return toff;
}


uint32_t TMC2208Base::encode_toff(uint8_t toff)
{
	uint32_t data = (CHOPCONF.data & ~TOFF_BM);
	uint32_t _toff = (uint32_t)toff;

	data |= (_toff << TOFF_BP);

	return data;
}




uint8_t TMC2208Base::decode_ihold(uint32_t data)
{
	uint8_t ihold = (data & IHOLD_BM) >> IHOLD_BP;
	return ihold;
}

uint32_t TMC2208Base::encode_ihold(uint8_t ihold)
{
	uint32_t data = (IHOLD_IRUN.data & ~IHOLD_BM);
	data |= (ihold << IHOLD_BP);

	return data;
}

uint8_t TMC2208Base::decode_irun(uint32_t data)
{
	uint8_t irun = (data & IRUN_BM) >> IRUN_BP;
	return irun;
}

uint32_t TMC2208Base::encode_irun(uint8_t irun)
{
	uint32_t data = (IHOLD_IRUN.data & ~IRUN_BM);
	data |= (irun << IRUN_BP);

	return data;
}

uint8_t TMC2208Base::decode_iholddelay(uint32_t data)
{
	uint8_t iholddelay = (data & IHOLDDELAY_BM) >> IHOLDDELAY_BP;
	return iholddelay;
}

uint32_t TMC2208Base::encode_iholddelay(uint8_t iholddelay)
{
	uint32_t data = (IHOLD_IRUN.data & ~IHOLDDELAY_BM);
	data |= (iholddelay << IHOLDDELAY_BP);

	return data;
}

uint32_t TMC2208Base::decode_dataframe(uint8_t* buf)
{
	uint32_t data = 0;

	data |= (uint32_t)buf[DATA0_IDX] << 24;
	data |= (uint32_t)buf[DATA1_IDX] << 16;
	data |= (uint32_t)buf[DATA2_IDX] << 8;
	data |= (uint32_t)buf[DATA3_IDX] << 0;

	return data;
}


void TMC2208Base::encode_dataframe(uint8_t* buf, uint32_t data)
{
	buf[DATA0_IDX] = (data & 0xFF000000) >> 24;
	buf[DATA1_IDX] = (data & 0x00FF0000) >> 16;
	buf[DATA2_IDX] = (data & 0x0000FF00) >> 8;
	buf[DATA3_IDX] = (data & 0x000000FF) >> 0;
}







bool TMC2208Base::err_get(TMC2208Err* p_err)
{
	if (*p_err == TMC2208Err::NONE) {
		return false;
	}
	return true;
}

void TMC2208Base::err_clear(TMC2208Err* p_err)
{
	*p_err = TMC2208Err::NONE;
}

