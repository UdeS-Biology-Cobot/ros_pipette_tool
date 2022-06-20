#pragma once

#include <actionlib/server/simple_action_server.h>
#include <robotic_tool/PipetteCommand.h>
#include <robotic_tool/PipetteCommandAction.h>
#include <ros/ros.h>

#include <robotic_tool/pipette_tool/protocol/posix/pt_master_protocol.h>
#include <robotic_tool/pipette_tool/posix/pipette_tool_control.h>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

class PipetteToolActionServer
{
public:
	PipetteToolActionServer(std::string name, PipetteToolControl& ptc);

	~PipetteToolActionServer();

	void goalCB();
	void preemptCB();

protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<robotic_tool::PipetteCommandAction> as_;
	std::string action_name_;
	int data_count_;
	float sum_, sum_sq_;

	// robotic_tool::PipetteCommand goal_;
	robotic_tool::PipetteCommandFeedback feedback_;
	robotic_tool::PipetteCommandResult result_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

private:
	PipetteToolControl ptc_;
};
