/*
 * drv8880_master_protocol.cpp
 *
 *  Created on: Jun 6, 2018
 *      Author: robotic_tools
 */

#include <robotic_tools/error_handle/error_handle.h>
#include <robotic_tools/pipette_tool/protocol/pipette_tool_master_protocol.h>


#include <stdio.h>


PipetteToolMasterProtocol::PipetteToolMasterProtocol(uint8_t slave_addr, boost::function<uint8_t(uint8_t*, uint8_t)> const &cb_write , boost::function<uint8_t(void)> const &cb_read)
:PipetteToolBaseProtocol(slave_addr), cb_write(cb_write), cb_read(cb_read)
{
	get_conf();
	set_driver_default();


	/* Create buffer with equal size of buffer in microcontroller  */
	TRAJECTORY_BUF_SIZE = trajectory_reg.get_buf_len();
	trajectory_buf = new uint16_t[TRAJECTORY_BUF_SIZE];
	/* TODO check if enough memory */
}

void PipetteToolMasterProtocol::set_driver_default()
{
	uint32_t cur_datareg;
	uint32_t data;
	uint8_t* cmd;
	uint32_t offset_reg;

	switch (driver_type)
	{
		case StepperDriver::Type::DRV8880:



			/* Set Microsteps */
			cur_datareg = ((DRV8880MasterRegister*)driver_reg)->get_register(DRV8880MasterRegister::Registers::CONFIG);

			data = ((DRV8880MasterRegister*)driver_reg)->encode_microstep_mode(DRV8880::MicrostepMode::SIXTEENTH_STEP, cur_datareg);
			//data = ((DRV8880MasterRegister*)driver_reg)->encode_current(DRV8880::Current::PERCENT_100, data);


			offset_reg = get_register_offset(RegisterType::STEPPER_DRIVER, (uint8_t)DRV8880MasterRegister::Registers::CONFIG);
			cmd = command(offset_reg, data);
			cmd_write(cmd);
			break;
	}

}

void PipetteToolMasterProtocol::set_direction(PipetteTool::Direction dir)
{
	uint8_t* cmd;
	uint32_t data;
	uint32_t offset_reg;
	uint32_t cur_datareg;

	Leadscrew::ThreadDir thread_dir = leadscrew_reg.get_thread_dir();
	DRV8880::Direction driver_dir = ((DRV8880MasterRegister*)driver_reg)->get_direction();

	if (dir == PipetteTool::Direction::DISPENSE) {

		if (thread_dir == Leadscrew::ThreadDir::RIGHT_HAND && driver_dir == DRV8880::Direction::CLOCKWISE) {
			/* Do nothing good direction */
		}
		else if (thread_dir == Leadscrew::ThreadDir::LEFT_HAND && driver_dir == DRV8880::Direction::COUNTER_CLOCKWISE){
			/* Do nothing good direction */
		}
		else {
			cur_datareg = ((DRV8880MasterRegister*)driver_reg)->get_register(DRV8880MasterRegister::Registers::CONFIG);

			if (thread_dir == Leadscrew::ThreadDir::RIGHT_HAND) {
				data = ((DRV8880MasterRegister*)driver_reg)->encode_direction(DRV8880::Direction::CLOCKWISE, cur_datareg);
			}
			else {
				data = ((DRV8880MasterRegister*)driver_reg)->encode_direction(DRV8880::Direction::COUNTER_CLOCKWISE, cur_datareg);
			}

			offset_reg = get_register_offset(RegisterType::STEPPER_DRIVER, (uint8_t)DRV8880MasterRegister::Registers::CONFIG);
			cmd = command(offset_reg, data);
			cmd_write(cmd);
		}
	}
	else if (dir == PipetteTool::Direction::ASPIRATE) {
		if (thread_dir == Leadscrew::ThreadDir::RIGHT_HAND && driver_dir == DRV8880::Direction::COUNTER_CLOCKWISE) {
			/* Do nothing good direction */
		}
		else if (thread_dir == Leadscrew::ThreadDir::LEFT_HAND && driver_dir == DRV8880::Direction::CLOCKWISE){
			/* Do nothing good direction */
		}
		else {
			cur_datareg = ((DRV8880MasterRegister*)driver_reg)->get_register(DRV8880MasterRegister::Registers::CONFIG);

			if (thread_dir == Leadscrew::ThreadDir::RIGHT_HAND) {
				data = ((DRV8880MasterRegister*)driver_reg)->encode_direction(DRV8880::Direction::COUNTER_CLOCKWISE, cur_datareg);
			}
			else {
				data = ((DRV8880MasterRegister*)driver_reg)->encode_direction(DRV8880::Direction::CLOCKWISE, cur_datareg);
			}

			offset_reg = get_register_offset(RegisterType::STEPPER_DRIVER, (uint8_t)DRV8880MasterRegister::Registers::CONFIG);
			cmd = command(offset_reg, data);
			cmd_write(cmd);
		}
	}
	else{
		/* TODO set error */

	}
}






PipetteToolMasterProtocol::~PipetteToolMasterProtocol()
{
	switch (driver_type)
	{
		case StepperDriver::Type::DRV8880:
			delete ((DRV8880MasterRegister*)driver_reg);
			break;
	}

	if (trajectory_buf != nullptr) {
		delete[] trajectory_buf;
	}
}













void PipetteToolMasterProtocol::get_conf()
{
	uint8_t* cmd;
	uint8_t reg = 0;
	uint8_t offset_reg;


	/* Pipette tool */
	for (reg = 0; reg < (uint8_t)pipette_reg.Registers::SIZE; reg++) {
		offset_reg = reg + OFFSET_REG_PIPETTE_TOOL;
		cmd = command(offset_reg);
		cmd_write(cmd);
	}

	/* Get driver */
	driver_type = StepperDriver::Type(pipette_reg.get_register(PipetteToolBaseRegister::Registers::DRIVER));

	switch (driver_type)
	{
		case StepperDriver::Type::DRV8880:
			driver_reg = new DRV8880MasterRegister();
			break;
	}

	/* Stepper motor */
	for (reg = 0; reg < (uint8_t)motor_reg.Registers::SIZE; reg++) {
		offset_reg = reg + OFFSET_REG_STEPPER_MOTOR;
		cmd = command(offset_reg);
		cmd_write(cmd);
	}

	/* Stepper driver */
	switch (driver_type)
	{
		case StepperDriver::Type::DRV8880:
			for (reg = 0; reg < (uint8_t)((DRV8880MasterRegister*)driver_reg)->Registers::SIZE; reg++) {
				offset_reg = reg + OFFSET_REG_STEPPER_DRIVER;
				cmd = command(offset_reg);
				cmd_write(cmd);
			}
			break;
	}

	/* Leadscrew */
	for (reg = 0; reg < (uint8_t)leadscrew_reg.Registers::SIZE; reg++) {
		offset_reg = reg + OFFSET_REG_LEADSCREW;
		cmd = command(offset_reg);
		cmd_write(cmd);
	}

	/* Trajectory */
	for (reg = 0; reg < (uint8_t)trajectory_reg.Registers::SIZE; reg++) {
		offset_reg = reg + OFFSET_REG_TRAJECTORY;
		cmd = command(offset_reg);
		cmd_write(cmd);

	}
}


void PipetteToolMasterProtocol::cmd_move(PipetteTool::Direction dir, double uL, Trajectory::Profile profile_type, uint32_t params[])
{
	double distance_per_step;
	double travel_distance;
	uint32_t steps;
	uint32_t f_min;
	uint32_t f_max;
	uint32_t slope;

	uint8_t* cmd;
	uint32_t data;

	float lead;
	uint16_t microstep;
	uint32_t motor_steps;
	uint32_t f_cpu;


	lead = leadscrew_reg.get_lead();



	switch (driver_type)
	{
		case StepperDriver::Type::DRV8880:
			microstep = (uint16_t)((DRV8880MasterRegister*)driver_reg)->get_microstep();
			break;
	}

	motor_steps = motor_reg.get_steps();
	f_cpu = trajectory_reg.get_register(TrajectoryBaseRegister::Registers::CPU_FREQUENCY);


	/* Calculate distance required for desired volume */
	travel_distance = get_distance(uL);

	/* Calculate number of steps needed for generating trajectory */
	distance_per_step = lead / (motor_steps*microstep);

	steps = travel_distance / distance_per_step;

	f_min = params[1];
	f_max = params[2] * microstep;
	//slope = params[3] * microstep;
	//slope = 175;
	slope = 175;

	/* Send command */
	uint32_t buf_len;
	uint8_t offset_reg;


	/* Set direction */
	set_direction(dir);


#ifdef TEMP_TRAJECTORY

	switch (profile_type) {

		case Trajectory::Profile::CONCAVE_SCURVE_VEL:
		{
			ProfileConcaveScurveVel profile = ProfileConcaveScurveVel(trajectory_buf, TRAJECTORY_BUF_SIZE);

			/* Populate buffer with profile */
			buf_len = profile.create(f_cpu, steps, f_min, f_max, slope);
			break;
		}
		default:
			/* TODO set error */
			p_err->set(TrajectoryErrCode::TRAJECTORY_NOT_IMPLEMENTED);
	}

	if (p_err->get()) {
		std::string s = p_err->getCodeString(p_err->getErrCode());
		std::cout << s << std::endl;
		return;
	}

	offset_reg = get_register_offset(PipetteToolBaseProtocol::RegisterType::TRAJECTORY, (uint8_t)TrajectoryBaseRegister::Registers::BUF_NEW);
	cmd = command(offset_reg, trajectory_buf[0]);
	cmd_write(cmd);


	offset_reg = get_register_offset(PipetteToolBaseProtocol::RegisterType::TRAJECTORY, (uint8_t)TrajectoryBaseRegister::Registers::BUF_ADD);
	for (uint16_t i = 1; i < buf_len; i++)
	{
		cmd = command(offset_reg, trajectory_buf[i]);
		cmd_write(cmd);
	}
#endif


	ProfileConcaveScurveVel profile = ProfileConcaveScurveVel(trajectory_buf, TRAJECTORY_BUF_SIZE);
	buf_len = profile.create(f_cpu, steps, f_min, f_max, slope);



	switch (profile_type) {

		case Trajectory::Profile::CONCAVE_SCURVE_VEL:
		{

			offset_reg = get_register_offset(PipetteToolBaseProtocol::RegisterType::TRAJECTORY, (uint8_t)TrajectoryBaseRegister::Registers::PROFILE);
			data = (uint8_t)trajectory_reg.encode_profile(Trajectory::Profile::CONCAVE_SCURVE_VEL);
			cmd = command(offset_reg, data);
			cmd_write(cmd);


			//GENERATE_WITH_PARAMS
			offset_reg = get_register_offset(PipetteToolBaseProtocol::RegisterType::TRAJECTORY, (uint8_t)TrajectoryBaseRegister::Registers::PARAM0);
			cmd = command(offset_reg, steps);
			cmd_write(cmd);

			offset_reg = get_register_offset(PipetteToolBaseProtocol::RegisterType::TRAJECTORY, (uint8_t)TrajectoryBaseRegister::Registers::PARAM1);
			cmd = command(offset_reg, f_min);
			cmd_write(cmd);

			offset_reg = get_register_offset(PipetteToolBaseProtocol::RegisterType::TRAJECTORY, (uint8_t)TrajectoryBaseRegister::Registers::PARAM2);
			cmd = command(offset_reg, f_max);
			cmd_write(cmd);

			offset_reg = get_register_offset(PipetteToolBaseProtocol::RegisterType::TRAJECTORY, (uint8_t)TrajectoryBaseRegister::Registers::PARAM3);
			cmd = command(offset_reg, slope);
			cmd_write(cmd);


			offset_reg = get_register_offset(PipetteToolBaseProtocol::RegisterType::TRAJECTORY, (uint8_t)TrajectoryBaseRegister::Registers::COMMAND);
			data = trajectory_reg.encode_command(TrajectoryBaseRegister::Command::GENERATE_WITH_PARAMS);
			cmd = command(offset_reg, data);
			cmd_write(cmd);
			break;
		}
		default:
			/* TODO set error */
			p_err->set(TrajectoryErrCode::TRAJECTORY_NOT_IMPLEMENTED);
	}

	if (p_err->get()) {
		std::string s = p_err->getCodeString(p_err->getErrCode());
		std::cout << s << std::endl;
		return;
	}



	/* Start Trajectory*/

	offset_reg = get_register_offset(PipetteToolBaseProtocol::RegisterType::TRAJECTORY, (uint8_t)TrajectoryBaseRegister::Registers::COMMAND);
	data = trajectory_reg.encode_command(TrajectoryBaseRegister::Command::RUN_TRAJECTORY);
	cmd = command(offset_reg, data);
	cmd_write(cmd);

	/* TODO check with encoder */
}


double PipetteToolMasterProtocol::get_distance(double uL)
{

	PipetteTool::Type pipette_type = PipetteTool::Type(pipette_reg.get_register(PipetteToolBaseRegister::Registers::TYPE));

	switch (pipette_type) {
		case PipetteTool::Type::GILSON_P200:
			return ((uL + 1.918) / 12.499);

	}
	/*Throw error if passed this */
	return 0.0;
}

void PipetteToolMasterProtocol::setFrameRegister(uint8_t offset_reg, RegisterRW rw)
{
	datagram[REGISTER_IDX] = 0;
	datagram[REGISTER_IDX] |= offset_reg;
	datagram[REGISTER_IDX] |= encode_register_rw(rw);
}


uint8_t* PipetteToolMasterProtocol::command(uint8_t offset_reg)
{
	/* SYNC + SLAVE + REGISTER 			*/
	setSyncFrame();
	setSlaveFrame();
	setFrameRegister(offset_reg, RegisterRW::REGISTER_READ);

	/* DATA0 + DATA1 + DATA2 + DATA3 	*/
	setDataFrame(0);

	/* CRC 								*/
	crc8_atm(datagram, DATAGRAM_SIZE);

	return datagram;
}

uint8_t* PipetteToolMasterProtocol::command(uint8_t offset_reg, uint32_t data)
{
	/* SYNC + SLAVE + REGISTER 			*/
	setSyncFrame();
	setSlaveFrame();
	setFrameRegister(offset_reg, RegisterRW::REGISTER_WRITE);

	/* DATA0 + DATA1 + DATA2 + DATA3 	*/
	setDataFrame(data);

	/* CRC 								*/
	crc8_atm(datagram, DATAGRAM_SIZE);

	return datagram;
}



#ifdef MACRO

uint8_t* PipetteToolMasterProtocol::cmd_stepper_motor(uint8_t reg, RegisterRW rw)
{
	uint32_t data;

	/* SYNC + SLAVE + REGISTER 			*/
	setSyncFrame();
	setSlaveFrame();
	setFrameRegisterStepperMotor(reg, rw);

	/* DATA0 + DATA1 + DATA2 + DATA3 	*/


	data = 0;
	/*TODO check for error 				*/
	setDataFrame(data);

	/* CRC 								*/
	crc8_atm(datagram, DATAGRAM_SIZE);

	return datagram;
}

uint8_t* PipetteToolMasterProtocol::cmd_pipette_tool(uint8_t reg, RegisterRW rw)
{
	uint32_t data;

	/* SYNC + SLAVE + REGISTER 			*/
	setSyncFrame();
	setSlaveFrame();
	setFrameRegisterPipetteTool(reg, rw);

	/* DATA0 + DATA1 + DATA2 + DATA3 	*/

	data = 0;
	/*TODO check for error 				*/
	setDataFrame(data);

	/* CRC 								*/
	crc8_atm(datagram, DATAGRAM_SIZE);

	return datagram;
}

uint8_t* PipetteToolMasterProtocol::cmd_stepper_driver(uint8_t reg, RegisterRW rw)
{
	uint32_t data;

	/* SYNC + SLAVE + REGISTER 			*/
	setSyncFrame();
	setSlaveFrame();
	setFrameRegisterStepperDriver(reg, rw);

	/* DATA0 + DATA1 + DATA2 + DATA3 	*/


	data = 0;
	/*TODO check for error 				*/
	setDataFrame(data);

	/* CRC 								*/
	crc8_atm(datagram, DATAGRAM_SIZE);

	return datagram;
}
#endif


void PipetteToolMasterProtocol::cmd_write(uint8_t* buf)
{
	uint8_t byte_ctr = 0;

	while (byte_ctr != DATAGRAM_SIZE) {
		byte_ctr += cb_write(buf, DATAGRAM_SIZE);
	}

	bool response = false;
	uint8_t byte;
	while (!response) {
		byte = cb_read();
		response = read_response(byte);
	}
}


void PipetteToolMasterProtocol::cmd_wrtie_only(uint8_t* buf)
{
	uint8_t byte_ctr = 0;

	while (byte_ctr != DATAGRAM_SIZE) {
		byte_ctr += cb_write(buf, DATAGRAM_SIZE);
	}
}

bool PipetteToolMasterProtocol::read_response(uint8_t data) {
	bool valid_datagram;
	valid_datagram = read_datagram(data);

	if (!valid_datagram) {
		return false;
	}
	uint8_t reg;
	uint8_t offset_reg = (datagram[REGISTER_IDX]  & REGISTER_BM) >> REGISTER_BP;
	uint32_t data_frame = unpackDataFrame();


	if (offset_reg >= OFFSET_REG_PIPETTE_TOOL && offset_reg < (OFFSET_REG_PIPETTE_TOOL + SIZE_REG_PIPETTE_TOOL))
	{
		reg = offset_reg - OFFSET_REG_PIPETTE_TOOL;
		pipette_reg.set_register(PipetteToolBaseRegister::Registers(reg), data_frame);
	}
	else if (offset_reg >= OFFSET_REG_STEPPER_MOTOR && offset_reg < (OFFSET_REG_STEPPER_MOTOR + SIZE_REG_STEPPER_MOTOR))
	{
		reg = offset_reg - OFFSET_REG_STEPPER_MOTOR;
		motor_reg.set_register(StepperMotorBaseRegister::Registers(reg), data_frame);
	}
	else if (offset_reg >= OFFSET_REG_STEPPER_DRIVER && offset_reg < (OFFSET_REG_STEPPER_DRIVER + SIZE_REG_STEPPER_DRIVER))
	{
		reg = offset_reg - OFFSET_REG_STEPPER_DRIVER;
		switch (driver_type)
		{
			case StepperDriver::Type::DRV8880:
				((DRV8880MasterRegister*)driver_reg)->set_register(DRV8880MasterRegister::Registers(reg), data_frame);
				break;
		}
	}
	else if (offset_reg >= OFFSET_REG_TRAJECTORY && offset_reg < (OFFSET_REG_TRAJECTORY + SIZE_REG_TRAJECTORY))
	{
		reg = offset_reg - OFFSET_REG_TRAJECTORY;
		trajectory_reg.set_register(TrajectoryBaseRegister::Registers(reg), data_frame);
	}
	else if (offset_reg >= OFFSET_REG_LEADSCREW && offset_reg < (OFFSET_REG_LEADSCREW + SIZE_REG_LEADSCREW))
	{
		reg = offset_reg - OFFSET_REG_LEADSCREW;
		leadscrew_reg.set_register(LeadscrewBaseRegister::Registers(reg), data_frame);
	}
	else if (offset_reg >= OFFSET_REG_ERROR && offset_reg < (OFFSET_REG_ERROR + SIZE_REG_ERROR))
	{


		std::string s = p_err->getCodeString(data_frame);
		std::cout << s << std::endl;

		reg = offset_reg - OFFSET_REG_ERROR;

		/* TODO print error */
	}
	else {
		/* TODO change error */
		p_err->set(PwmPfcErrCode::INVALID_FREQUENCY_FOR_TOP_CALCULATION);
		//goto error;
	}




	return true;
}






















