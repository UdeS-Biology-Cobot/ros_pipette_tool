
#include <ros_robotic_tools/pipette_tool_action_server.h>

PipetteToolActionServer::PipetteToolActionServer(std::string name, PipetteToolControl& ptc)
  : as_(nh_, name, false), action_name_(name), ptc_(ptc) {
	// register the goal and feeback callbacks
	as_.registerGoalCallback(boost::bind(&PipetteToolActionServer::goalCB, this));
	as_.registerPreemptCallback(boost::bind(&PipetteToolActionServer::preemptCB, this));

	// subscribe to the data topic of interest
	as_.start();
}

PipetteToolActionServer::~PipetteToolActionServer(void) {}

void PipetteToolActionServer::goalCB() {
	ros_robotic_tools::PipetteCommand goal = as_.acceptNewGoal()->command;

	ros_robotic_tools::PipetteCommandResult res;
	res.success = true;
	res.err = 0;

	PipetteToolControlErr err = PipetteToolControlErr::NONE;
	PipetteToolControlErr* p_err = &err;

	switch (goal.action) {
		case goal.ACTION_HOMING:
			ptc_.homing(goal.velocity, p_err);
			if (ptc_.err_get(p_err)) {
				res.success = false;
				res.err = (uint32_t)*p_err;
				ROS_ERROR("Homing failed: %d", res.err);
			}
			break;
		case goal.ACTION_MOVE_TO_1ST_STOP:
			ptc_.move_to_first_stop(goal.volume, goal.velocity, p_err);
			if (ptc_.err_get(p_err)) {
				res.success = false;
				res.err = (uint32_t)*p_err;
				ROS_ERROR("Move To First Stop failed: %d", res.err);
			}
			break;
		case goal.ACTION_MOVE_TO_2ND_STOP:
			ptc_.move_to_second_stop(goal.velocity, p_err);
			if (ptc_.err_get(p_err)) {
				res.success = false;
				res.err = (uint32_t)*p_err;
				ROS_ERROR("Move To Second Stop failed: %d", res.err);
			}
			break;
		case goal.ACTION_ASPIRATE:
			ptc_.aspirate(goal.volume, goal.velocity, p_err);
			if (ptc_.err_get(p_err)) {
				res.success = false;
				res.err = (uint32_t)*p_err;
				ROS_ERROR("Aspirate failed: %d", res.err);
			}
			break;
		case goal.ACTION_ASPIRATE_SEQ:
			ptc_.aspirate_sequence(goal.volume, goal.velocity, goal.sequence, p_err);
			if (ptc_.err_get(p_err)) {
				res.success = false;
				res.err = (uint32_t)*p_err;
				ROS_ERROR("Aspirate Sequence failed: %d", res.err);
			}
			break;
		case goal.ACTION_DISPENSE:
			ptc_.dispense(goal.volume, goal.velocity, p_err);
			if (ptc_.err_get(p_err)) {
				ROS_INFO("Volume = %d", goal.volume);
				res.success = false;
				res.err = (uint32_t)*p_err;
				ROS_ERROR("Dispense failed: %d", res.err);
			}
			break;
		case goal.ACTION_DISPENSE_SEQ:
			ptc_.dispense_sequence(goal.volume, goal.velocity, goal.sequence, p_err);
			if (ptc_.err_get(p_err)) {
				res.success = false;
				res.err = (uint32_t)*p_err;
				ROS_ERROR("Dispense Sequence failed: %d", res.err);
			}
			break;
		case goal.ACTION_EJECT:
			ptc_.eject(goal.velocity, p_err);
			if (ptc_.err_get(p_err)) {
				res.success = false;
				res.err = (uint32_t)*p_err;
				ROS_ERROR("Eject failed: %d", res.err);
			}
			break;

		default:
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
	errors += !ros::param::get("~action_server_topic", action_server);

	ROS_INFO_STREAM("Serial device: " << device);
	ROS_INFO_STREAM("Action server topic: " << action_server);

	if (errors) {
		ROS_ERROR_STREAM("Somme paramters are missing...");
		return -1;
	}

	// TODO add rosparam
	boost::asio::io_service io;
	boost::asio::serial_port port(io);

	PtMasterProtocol pt(&port, baudrate, device);
	PipetteToolControlErr err_ptc_ = PipetteToolControlErr::NONE;

	PipetteToolControl ptc_(&pt);

	// Retrieve information
	ROS_INFO("Retrieving Pipette information...\n");

	double max_speed = ptc_.get_max_speed(&err_ptc_);
	if (ptc_.err_get(&err_ptc_)) {
		ROS_ERROR_STREAM("PipetteToolControl error code = " << (uint32_t)err_ptc_);
		return -1;
	}

	ROS_INFO("==== Pipette Information ====");
	ROS_INFO_STREAM("Max speed (m/s) = " << max_speed);
	ROS_INFO("=============================\n");

	ROS_INFO("Action server started");
	PipetteToolActionServer as(action_server, ptc_);
	ros::spin();

	return 0;
}
