#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros_robotic_tools/PipetteCommandRawAction.h>
#include <ros_robotic_tools/PipetteCommandRaw.h>

bool validate_action(actionlib::SimpleActionClient<ros_robotic_tools::PipetteCommandRawAction>& ac, double timeout) {
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
	ros::init(argc, argv, "pipette_tool_raw_action_client_move_up_down");

	ros::NodeHandle pnh("~");

	double timeout = 10.0;
	std::string action_server_topic;
	double default_speed;
	double speed_upward;
	double speed_downward;
	double distance_upward_mm;
	double distance_downward_mm;

	// retrieve parameters
	pnh.param<std::string>("action_server_topic", action_server_topic, "udes_pipette");
	pnh.param<double>("default_speed", default_speed, 0.01);
	pnh.param<double>("speed_upward", speed_upward, 0.01);
	pnh.param<double>("speed_downward", speed_downward, 0.01);
	pnh.param<double>("distance_upward_mm", distance_upward_mm, 5);
	pnh.param<double>("distance_downward_mm", distance_downward_mm, 5);

	ros_robotic_tools::PipetteCommandRawGoal goal;

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<ros_robotic_tools::PipetteCommandRawAction> ac(action_server_topic, true);

	ROS_INFO("Waiting for action server to start...");
	ac.waitForServer();  // will wait for infinite time
	ROS_INFO("Action server started");

	// 1- HOMING
	ROS_INFO("Sending Goal: HOMING");
	goal.command.action = goal.command.ACTION_HOMING;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	// 2- Move to Tip
	ROS_INFO("Sending Goal: Move to Tip");
	goal.command.action = goal.command.ACTION_MOVE_TO_TIP;
	goal.command.velocity = default_speed;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	// 3- Move Upward
	ROS_INFO("Sending Goal: Move Upward");
	goal.command.action = goal.command.ACTION_MOVE_UPWARD;
	goal.command.position = distance_upward_mm;
	goal.command.velocity = speed_upward;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	// 4- Move Downward
	ROS_INFO("Sending Goal: Move Downward");
	goal.command.action = goal.command.ACTION_MOVE_DOWNWARD;
	goal.command.position = distance_downward_mm;
	goal.command.velocity = speed_downward;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	ROS_INFO("Successfull Forward Pipetting");
	return 0;
}
