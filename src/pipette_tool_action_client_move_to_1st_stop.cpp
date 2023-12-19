#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros_robotic_tools/PipetteCommandAction.h>
#include <ros_robotic_tools/PipetteCommand.h>

bool validate_action(actionlib::SimpleActionClient<ros_robotic_tools::PipetteCommandAction>& ac, double timeout) {
	// wait for the action to return
	if (ac.waitForResult(ros::Duration(timeout))) {
		actionlib::SimpleClientGoalState state = ac.getState();

		if (state.state_ == state.SUCCEEDED) {
			ROS_INFO("Action Succeeded");
			return true;
		} else {
			ROS_ERROR("Action finished: %s", state.toString().c_str());
			return false;
		}

	} else {
		ROS_ERROR("Action did not finish before the time out.");
		return false;
	}

	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "pipette_tool_action_client_move_to_1st_stop");

	ros::NodeHandle pnh("~");

	double timeout = 10.0;
	std::string action_server_topic;
	double speed;
	int volume;

	// retrieve parameters
	pnh.param<std::string>("action_server_topic", action_server_topic, "udes_pipette");
	pnh.param<double>("speed", speed, 0.01);
	pnh.param<int>("volume", volume, 0);

	ros_robotic_tools::PipetteCommandGoal goal;

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<ros_robotic_tools::PipetteCommandAction> ac(action_server_topic, true);

	ROS_INFO("Waiting for action server to start...");
	ac.waitForServer();  // will wait for infinite time
	ROS_INFO("Action server started");

	// Move to 1st Stop
	ROS_INFO_STREAM("Sending Goal: Move to 1st Stop @ " << speed << " m/s with an offset of " << volume << " nL");
	goal.command.action = goal.command.ACTION_MOVE_TO_1ST_STOP;
	goal.command.volume = volume;
	goal.command.velocity = speed;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	ROS_INFO("Successfull Move to 1st Stop");
	return 0;
}
