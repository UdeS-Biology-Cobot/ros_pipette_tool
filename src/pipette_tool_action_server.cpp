
#include <robotic_tool/pipette_tool_action_server.h>

PipetteToolActionServer::PipetteToolActionServer(std::string name, PipetteToolControl& ptc)
  : as_(nh_, name, false), action_name_(name), ptc_(ptc) {
	// register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&PipetteToolActionServer::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&PipetteToolActionServer::preemptCB, this));

	// subscribe to the data topic of interest
	ROS_INFO("Action server Starting");
	as_.start();
}

PipetteToolActionServer::~PipetteToolActionServer(void) {}

void PipetteToolActionServer::goalCB() {
	robotic_tool::PipetteCommand goal = as_.acceptNewGoal()->command;

	robotic_tool::PipetteCommandResult res;
	res.success = true;
	res.err = 0;

	PipetteToolControlErr err = PipetteToolControlErr::NONE;
	PipetteToolControlErr* p_err = &err;
	ROS_WARN("Pipette Command entered");

	if (goal.mode == goal.MODE_FORWARD) {
		switch (goal.action) {
			case goal.ACTION_HOMING:
				ptc_.homing(goal.velocity, p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Forward Homing failed: %d", res.err);
				}
				break;
			case goal.ACTION_INIT:
				ptc_.forward_init(goal.offset_nl, goal.velocity, p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Forward Init failed: %d", res.err);
				}
				break;
			case goal.ACTION_ASPIRATE:
				ptc_.forward_aspirate(goal.volume_nl, goal.velocity, p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Forward Aspirate failed: %d", res.err);
				}
				break;
			case goal.ACTION_ASPIRATE_SEQ:
				ptc_.forward_aspirate_sequence(goal.volume_nl, goal.velocity, goal.sequence, p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Forward Aspirate Sequence failed: %d", res.err);
				}
				break;
			case goal.ACTION_DISPENSE:
				ptc_.forward_dispense(goal.volume_nl, goal.offset_nl, goal.velocity, p_err);
				if (ptc_.err_get(p_err)) {
					ROS_INFO("Volume = %d", goal.volume_nl);
					ROS_INFO("Offset = %d", goal.offset_nl);
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Forward Dispense failed: %d", res.err);
				}
				break;
			case goal.ACTION_DISPENSE_SEQ:
				ptc_.forward_dispense_sequence(goal.volume_nl, goal.offset_nl, goal.velocity, goal.sequence, p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Forward Dispense Sequence failed: %d", res.err);
				}
				break;
			case goal.ACTION_EJECT:
				ptc_.eject(p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Forward Eject failed: %d", res.err);
				}
				break;

			default:
				ROS_ERROR("Mode is inexistant");
				res.success = false;
				res.err = 1;
		}
	} else if (goal.mode == goal.MODE_REVERSE) {
		switch (goal.action) {
			case goal.ACTION_HOMING:
				ptc_.homing(goal.velocity, p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Reverse Homing failed: %d", res.err);
				}
				break;
			case goal.ACTION_INIT:
				ptc_.reverse_init(goal.velocity, p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Reverse Init failed: %d", res.err);
				}
				break;
			case goal.ACTION_ASPIRATE:
				ptc_.reverse_aspirate(goal.volume_nl, goal.offset_nl, goal.velocity, p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Reverse Aspirate failed: %d", res.err);
				}
				break;
			case goal.ACTION_DISPENSE:
				ptc_.reverse_dispense(goal.volume_nl, goal.velocity, p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Reverse Dispense failed: %d", res.err);
				}
				break;
			case goal.ACTION_EJECT:
				ptc_.eject(p_err);
				if (ptc_.err_get(p_err)) {
					res.success = false;
					res.err = (uint32_t)*p_err;
					ROS_ERROR("Reverse Eject failed: %d", res.err);
				}
				break;

			default:
				ROS_ERROR("Mode is inexistant");
				res.success = false;
				res.err = 1;
		}

	} else {
		ROS_ERROR("Mode is inexistant");
		res.success = false;
		res.err = 1;
	}

	if (res.success) {
		as_.setSucceeded();
	} else {
		as_.setAborted(res);
	}
}

void PipetteToolActionServer::preemptCB() {
	ROS_INFO("%s: Preempted", action_name_.c_str());
	// set the action state to preempted
	as_.setPreempted();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pipette_tool_action_server");

	// Get params
	int baudrate;
	std::string device;
	std::string action_server;

	size_t errors = 0;
	errors += !ros::param::get("~baudrate", baudrate);
	errors += !ros::param::get("~device", device);
	errors += !ros::param::get("~action_server", action_server);

	ROS_ERROR_STREAM(device);
	ROS_ERROR_STREAM(action_server);

	if (errors) {
		ROS_ERROR_STREAM("Somme paramters are missing...");
		return -1;
	}

	// TODO add rosparam
	boost::asio::io_service io;
	boost::asio::serial_port port(io);

	PtMasterProtocol pt(&port, baudrate, device);
	PipetteToolControlErr err_ptc_ = PipetteToolControlErr::NONE;
	ROS_INFO("OHLALA1V");

	PipetteToolControl ptc_(&pt, &err_ptc_);
	if (ptc_.err_get(&err_ptc_)) {
		ROS_ERROR_STREAM("PipetteToolControl error code = " << (uint32_t)err_ptc_);
		return -1;
	}
	ROS_INFO("HACK");

	// HACK prepare forward pipette init

	ptc_.forward_init(20000, 0.01, &err_ptc_);
	ROS_INFO("HACKKKKKKK");
	if (ptc_.err_get(&err_ptc_)) {
		ROS_ERROR_STREAM("PipetteToolControl error code = " << (uint32_t)err_ptc_);
		return -1;
	}
	ROS_INFO("OHLALA");

	PipetteToolActionServer as(action_server, ptc_);
	ros::spin();

	return 0;
}
