/*
 * pipette_tool_control.cpp
 *
 *  Created on: Dec 12, 2018
 *      Author: biobot
 */

#if defined(__linux__)

#include <robotic_tool/pipette_tool/posix/pipette_tool_control.h>

// TODO delete
#include <iostream>

PipetteToolControl::PipetteToolControl(PtMasterProtocol* pt, PipetteToolControlErr* p_err)
: pt(pt)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;

	pt->move_to_tip(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}

	max_speed = pt->get_max_speed(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
	std::cout << "max_speed = " << max_speed << std::endl;


	max_volume_nl = pt->get_max_nl(&err_pt) ;
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}


void PipetteToolControl::forward_init(uint32_t offset_nl, double speed, PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;

	pt->move_to_tip(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}

	validate_volume(offset_nl, p_err);
	if (err_get(p_err)) {return;}

	pt->aspirate(offset_nl, speed, &err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}

void PipetteToolControl::forward_aspirate(uint32_t nl, double speed, PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;

	validate_volume(nl, p_err);
	if (err_get(p_err)) {return;}

	pt->aspirate(nl, speed, &err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}


void PipetteToolControl::forward_dispense(uint32_t nl, uint32_t offset_nl, double speed, PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;
	uint32_t volume = nl + offset_nl;

	validate_volume(-(int32_t)volume, p_err);
	if (err_get(p_err)) {return;}

	pt->dispense(volume, speed, &err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}


void PipetteToolControl::reverse_init(double speed, PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;

	pt->move_to_tip(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}


void PipetteToolControl::reverse_aspirate(uint32_t nl, uint32_t offset_nl, double speed, PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;
	uint32_t volume = nl + offset_nl;

	validate_volume((int32_t)volume, p_err);
	if (err_get(p_err)) {return;}

	pt->aspirate(volume, speed, &err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}


void PipetteToolControl::reverse_dispense(uint32_t nl, double speed, PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;

	validate_volume(-(int32_t)nl, p_err);
	if (err_get(p_err)) {return;}

	pt->dispense(nl, speed, &err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}


void PipetteToolControl::eject(PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;

	pt->eject_tip(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}

	pt->move_to_tip(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}

void PipetteToolControl::aspirate(uint32_t nl, double speed, PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;

	validate_volume((int32_t)nl, p_err);
	if (err_get(p_err)) {return;}

	pt->aspirate(nl, speed, &err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}


void PipetteToolControl::dispense(uint32_t nl, double speed, PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;

	validate_volume(-(int32_t)nl, p_err);
	if (err_get(p_err)) {return;}

	pt->dispense(nl, speed, &err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}
}

uint32_t PipetteToolControl::get_max_volume_nl(PipetteToolControlErr* p_err)
{
	return max_volume_nl;
}





double PipetteToolControl::get_max_speed()
{
	return max_speed;
}


bool PipetteToolControl::err_get(PipetteToolControlErr* p_err)
{
	if (*p_err == PipetteToolControlErr::NONE) {
		return false;
	}
	return true;
}

void PipetteToolControl::err_clear(PipetteToolControlErr* p_err)
{
	*p_err = PipetteToolControlErr::NONE;
}


uint32_t PipetteToolControl::get_rem_aspirate_vol_nl(PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;


	uint32_t vol = pt->get_rem_aspirate_vol_nl(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return 0;
	}
	return vol;
}


uint32_t PipetteToolControl::get_rem_dispense_vol_nl(PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;


	uint32_t vol = pt->get_rem_dispense_vol_nl(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return 0;
	}
	return vol;
}






void PipetteToolControl::validate_volume(int32_t volume, PipetteToolControlErr* p_err)
{
	PtProtocolErr err_pt = PtProtocolErr::NONE;

	uint32_t rem_dispense_vol_nl = pt->get_rem_dispense_vol_nl(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}

	uint32_t rem_aspirate_vol_nl = pt->get_rem_aspirate_vol_nl(&err_pt);
	if (pt->err_get(&err_pt)) {
		*p_err = PipetteToolControlErr::PT_MASTER_PROTOCOL;
		return;
	}

	if (volume < 0) {
		if ((int32_t)rem_dispense_vol_nl + volume < 0) {
			*p_err = PipetteToolControlErr::VOLUME_OUT_OF_BOUND;
			return;
		}
	} else {
		if ((int32_t)rem_aspirate_vol_nl - volume < 0) {
			*p_err = PipetteToolControlErr::VOLUME_OUT_OF_BOUND;
			return;
		}
	}
}

#endif	// __linux__
