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
	ros::init(argc, argv, "pipette_tool_action_client_forward_pipetting");

	ros::NodeHandle pnh("~");

	double timeout = 10.0;
	std::string action_server_topic;
	double default_speed;
	double aspirate_speed;
	double dispense_speed;
	double purge_speed;
	double eject_speed;

	// retrieve parameters
	pnh.param<std::string>("action_server_topic", action_server_topic, "udes_pipette");
	pnh.param<double>("pipetting_default_speed", default_speed, 0.01);
	pnh.param<double>("pipetting_aspirate_speed", aspirate_speed, 0.01);
	pnh.param<double>("pipetting_dispense_speed", dispense_speed, 0.01);
	pnh.param<double>("pipetting_purge_speed", purge_speed, 0.01);
	pnh.param<double>("pipetting_eject_speed", eject_speed, 0);

	ros_robotic_tools::PipetteCommandGoal goal;

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<ros_robotic_tools::PipetteCommandAction> ac(action_server_topic, true);

	ROS_INFO("Waiting for action server to start...");
	ac.waitForServer();  // will wait for infinite time
	ROS_INFO("Action server started");

	// 1- HOMING
	ROS_INFO("Sending Goal: HOMING");
	goal.command.action = goal.command.ACTION_HOMING;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	// 2- Move to First Stop
	ROS_INFO("Sending Goal: Move to First Stop");
	goal.command.action = goal.command.ACTION_MOVE_TO_1ST_STOP;
	goal.command.volume = 10000;  // First Stop, 10 uL upward offset from 2nd stop for purging liquid (step 5)
	goal.command.velocity = default_speed;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	// 3- Aspirate 50 uL
	ROS_INFO("Sending Goal: Aspirate 50 uL");
	goal.command.action = goal.command.ACTION_ASPIRATE;
	goal.command.volume = 50000;  // 50 uL
	goal.command.velocity = aspirate_speed;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	// 4- Dispense 50 uL
	ROS_INFO("Sending Goal: Dispense 50 uL");
	goal.command.action = goal.command.ACTION_DISPENSE;
	goal.command.volume = 50000;  // 50 uL
	goal.command.velocity = dispense_speed;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	// 5- Purge liquid (OPTION 1)
	ROS_INFO("Sending Goal: Purge");
	goal.command.action = goal.command.ACTION_MOVE_TO_2ND_STOP;
	goal.command.volume = 0;  // Second STOP
	goal.command.velocity = purge_speed;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	// 5- Purge liquid (OPTION 2)
	/*
	ROS_INFO("Sending Goal: Purge");
	goal.command.action = goal.command.DISPENSE;
	goal.command.volume = 10000; // Remaining volume
	goal.command.velocity = purge_speed;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
	   return 0;
	*/

	// 6- Eject Tip
	ROS_INFO("Sending Goal: Eject Tip");
	goal.command.action = goal.command.ACTION_EJECT;
	goal.command.velocity = eject_speed;
	ac.sendGoal(goal);

	if (!validate_action(ac, timeout))
		return 0;

	ROS_INFO("Successfull Forward Pipetting");
	return 0;
}
