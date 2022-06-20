/*
 * pipette_tool_control.h
 *
 *  Created on: Dec 12, 2018
 *      Author: biobot
 */

#ifndef PIPETTE_TOOL_POSIX_PIPETTE_TOOL_CONTROL_H_
#define PIPETTE_TOOL_POSIX_PIPETTE_TOOL_CONTROL_H_

#if defined(__linux__)

#include <robotic_tool/pipette_tool/protocol/posix/pt_master_protocol.h>

enum class PipetteToolControlErr : uint32_t
{

	NONE,

	PT_MASTER_PROTOCOL,

	VOLUME_OUT_OF_BOUND,

};

class PipetteToolControl
{
public:
	enum class DispenseTechnique : uint8_t
	{
		FORWARD,
		REVERSE
	};

	PipetteToolControl(PtMasterProtocol* pt, PipetteToolControlErr* p_err);
	void homing(double speed, PipetteToolControlErr* p_err);

	void forward_init(uint32_t offset_nl, double speed, PipetteToolControlErr* p_err);
	void forward_aspirate(uint32_t nl, double speed, PipetteToolControlErr* p_err);
	void forward_dispense(uint32_t nl, uint32_t offset_nl, double speed, PipetteToolControlErr* p_err);
	void forward_aspirate_sequence(uint32_t nl, double speed, int seq, PipetteToolControlErr* p_err);
	void forward_dispense_sequence(uint32_t nl, uint32_t offset_nl, double speed, int seq, PipetteToolControlErr* p_err);

	void reverse_init(double speed, PipetteToolControlErr* p_err);
	void reverse_aspirate(uint32_t nl, uint32_t offset_nl, double speed, PipetteToolControlErr* p_err);
	void reverse_dispense(uint32_t nl, double speed, PipetteToolControlErr* p_err);

	void aspirate(uint32_t nl, double speed, PipetteToolControlErr* p_err);
	void dispense(uint32_t nl, double speed, PipetteToolControlErr* p_err);
	void eject(PipetteToolControlErr* p_err);

	uint32_t get_max_volume_nl(PipetteToolControlErr* p_err);
	double get_max_speed();

	uint32_t get_rem_aspirate_vol_nl(PipetteToolControlErr* p_err);
	uint32_t get_rem_dispense_vol_nl(PipetteToolControlErr* p_err);

	bool err_get(PipetteToolControlErr* p_err);
	void err_clear(PipetteToolControlErr* p_err);

private:
	// TODO maybe not necessary
	void validate_volume(int32_t volume, PipetteToolControlErr* p_err);

	PtMasterProtocol* pt;

	uint32_t max_volume_nl;
	double max_speed;
};

#endif  // __linux__

#endif /* PIPETTE_TOOL_POSIX_PIPETTE_TOOL_CONTROL_H_ */
