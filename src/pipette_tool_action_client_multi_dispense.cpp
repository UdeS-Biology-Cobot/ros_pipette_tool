#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ros_robotic_tools/PipetteCommandAction.h>
#include <ros_robotic_tools/PipetteCommand.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "pipette_tool_action_client_multi_dispense");

	ros::NodeHandle pnh("~");

	std::string tool_name;
	pnh.param<std::string>("tool_name", tool_name, "udes_pipette");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<ros_robotic_tools::PipetteCommandAction> ac(tool_name, true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer();  // will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	bool finished_before_timeout;
	// send a goal to the action
	ros_robotic_tools::PipetteCommandGoal goal;

	for (int i = 5; i < 100; i++) {
		// Init
		goal.command.mode = goal.command.MODE_FORWARD;
		goal.command.action = goal.command.ACTION_INIT;
		goal.command.velocity = 0.020;
		goal.command.volume_nl = 40000;
		goal.command.offset_nl = 60000;
		ac.sendGoal(goal);
		finished_before_timeout = ac.waitForResult(ros::Duration(100.0));

		if (finished_before_timeout) {
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s", state.toString().c_str());
		} else
			ROS_INFO("Action did not finish before the time out.");

		// Aspirate sequence
		goal.command.action = goal.command.ACTION_ASPIRATE_SEQ;
		goal.command.sequence = i;
		ac.sendGoal(goal);
		finished_before_timeout = ac.waitForResult(ros::Duration(100.0));

		if (finished_before_timeout) {
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s", state.toString().c_str());
		} else
			ROS_INFO("Action did not finish before the time out.");

		// Dispense sequence
		goal.command.action = goal.command.ACTION_DISPENSE_SEQ;
		goal.command.sequence = i;
		ac.sendGoal(goal);
		finished_before_timeout = ac.waitForResult(ros::Duration(100.0));

		if (finished_before_timeout) {
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s", state.toString().c_str());
		} else
			ROS_INFO("Action did not finish before the time out.");

		// Dispense sequence
		goal.command.action = goal.command.ACTION_EJECT;
		ac.sendGoal(goal);
		finished_before_timeout = ac.waitForResult(ros::Duration(100.0));

		if (finished_before_timeout) {
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s", state.toString().c_str());
		} else
			ROS_INFO("Action did not finish before the time out.");

		ros::Duration(10.0).sleep();
	}

	// exit
	return 0;
}
