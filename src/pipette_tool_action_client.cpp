#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robotic_tool/PipetteCommandAction.h>
#include <robotic_tool/PipetteCommand.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_robotiq_2f_gripper_action_server");

	ros::NodeHandle pnh("~");

	std::string tool_name;
	pnh.param<std::string>("tool_name", tool_name, "udes_pipette");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<robotic_tool::PipetteCommandAction> ac(tool_name, true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer();  // will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	robotic_tool::PipetteCommandGoal goal;
	goal.command.mode = goal.command.MODE_FORWARD;
	goal.command.action = goal.command.ACTION_INIT;
	goal.command.velocity = 0.005;
	goal.command.volume_nl = 200000;
	goal.command.offset_nl = 20000;
	ac.sendGoal(goal);

	// wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	} else
		ROS_INFO("Action did not finish before the time out.");

	goal.command.action = goal.command.ACTION_ASPIRATE;
	ac.sendGoal(goal);
	finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	} else
		ROS_INFO("Action did not finish before the time out.");

	for (int i = 0; i < 96; i++) {
		goal.command.action = goal.command.ACTION_DISPENSE;
		goal.command.velocity = 0.02;
		goal.command.volume_nl = 2080;
		goal.command.offset_nl = 0;
		ac.sendGoal(goal);
		finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

		if (finished_before_timeout) {
			actionlib::SimpleClientGoalState state = ac.getState();
			ROS_INFO("Action finished: %s", state.toString().c_str());
		} else
			ROS_INFO("Action did not finish before the time out.");
	}
	return 0;
	goal.command.action = goal.command.ACTION_DISPENSE;
	ac.sendGoal(goal);
	finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	} else
		ROS_INFO("Action did not finish before the time out.");

	goal.command.action = goal.command.ACTION_EJECT;
	ac.sendGoal(goal);
	finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

	if (finished_before_timeout) {
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	} else
		ROS_INFO("Action did not finish before the time out.");

	// exit
	return 0;
}
