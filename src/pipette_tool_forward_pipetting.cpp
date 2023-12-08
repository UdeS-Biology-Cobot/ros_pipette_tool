#include <ros/ros.h>
#include <string>

#include <robotic_tool/pipette_tool/protocol/posix/pt_master_protocol.h>
#include <robotic_tool/pipette_tool/posix/pipette_tool_control.h>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

// void forward(PipetteToolControl pt_ctrl, uint32_t nl, uint32_t offset_nl, double speed, PipetteToolControlErr* p_err)
// {

// 	pt_ctrl.forward_init(offset_nl, speed, p_err);
// 	if (pt_ctrl.err_get(p_err)) {
// 		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
// 		return;
// 	}

// 	pt_ctrl.forward_aspirate(nl, speed, p_err);
// 	if (pt_ctrl.err_get(p_err)) {
// 		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
// 		return;
// 	}

// 	pt_ctrl.forward_dispense(nl, offset_nl, speed, p_err);
// 	if (pt_ctrl.err_get(p_err)) {
// 		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
// 		return;
// 	}
// }

// void reverse(PipetteToolControl pt_ctrl, uint32_t nl, uint32_t offset_nl, double speed, PipetteToolControlErr* p_err)
// {

// 	pt_ctrl.reverse_init(speed, p_err);
// 	if (pt_ctrl.err_get(p_err)) {
// 		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
// 		return;
// 	}

// 	pt_ctrl.reverse_aspirate(nl, offset_nl, speed, p_err);
// 	if (pt_ctrl.err_get(p_err)) {
// 		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
// 		return;
// 	}

// 	pt_ctrl.reverse_dispense(nl, speed, p_err);
// 	if (pt_ctrl.err_get(p_err)) {
// 		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
// 		return;
// 	}
// }


// void eject(PipetteToolControl pt_ctrl, PipetteToolControlErr* p_err) {

// 	pt_ctrl.eject(p_err);
// 	if (pt_ctrl.err_get(p_err)) {
// 		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
// 		return;
// 	}
// }








int main(int argc, char **argv)
{
	ros::init(argc, argv, "pipette_tool_example");
	ros::NodeHandle pnh("~");

	// params
	std::string device;
	int baudrate;

	double timeout = 10.0;
	double default_speed;
	double aspirate_speed;
	double dispense_speed;
	double purge_speed;
	double eject_speed;

	// retrieve parameters
	pnh.param<std::string>("device", device, "/dev/ttyACM0");
	pnh.param<int>("baudrate", baudrate, 9600);
	pnh.param<double>("pipetting_default_speed", default_speed, 0.01);
	pnh.param<double>("pipetting_aspirate_speed", aspirate_speed, 0.01);
	pnh.param<double>("pipetting_dispense_speed", dispense_speed, 0.01);
	pnh.param<double>("pipetting_purge_speed", purge_speed, 0.01);
	pnh.param<double>("pipetting_eject_speed", eject_speed, 0);


	// Initialize Pipette Communication
	boost::asio::io_service io;
	boost::asio::serial_port port(io);

	PtProtocolErr err_pt = PtProtocolErr::NONE;
	PipetteToolControlErr err_pt_ctrl = PipetteToolControlErr::NONE;
	PtMasterProtocol pt = PtMasterProtocol(&port, baudrate, device);

	PipetteToolControl pt_ctrl(&pt);


	// 1- Homing
	ROS_INFO("Sending: Homing Sequence");
	pt_ctrl.homing(default_speed, &err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}

	// 2- Move to first stop
	ROS_INFO("Sending: Move to First Stop");
	pt_ctrl.move_to_first_stop(10000, // 10 uL
								default_speed,
								&err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}

	// 3- Get max volume @ current position
	ROS_INFO("Sending: Get Max Volume");
	double max_volume = pt_ctrl.get_max_volume(&err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}
	ROS_INFO_STREAM("Max Volume = " << max_volume << " nL");

	// 4- Get max speed
	ROS_INFO("Sending: Get Max Speed");
	double max_speed = pt_ctrl.get_max_speed(&err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}
	ROS_INFO_STREAM("Max Speed = " << max_speed << " mm/sec");

	// 5- Aspirate 50 uL
	ROS_INFO("Sending: Aspirate 50 uL");
	pt_ctrl.aspirate(50000, // 50 uL
					 aspirate_speed,
					 &err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}

	// 6- Dispense 50 uL
	ROS_INFO("Sending: Dispense 50 uL");
	pt_ctrl.dispense(50000, // 50 uL
					 dispense_speed,
					 &err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}

	// 7- Purge (OPTION 1)
	ROS_INFO("Sending: Purge");
	pt_ctrl.move_to_second_stop(purge_speed, &err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}

	// 7- Purge (OPTION 2)
	/*
	ROS_INFO("Sending: Purge");
	pt_ctrl.dispense(10000, // 10 uL
					 purge_speed,
					 &err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}
	*/

	// 8- Eject Tip
	ROS_INFO("Sending: Eject Tip");
	pt_ctrl.eject(eject_speed, &err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}

	return 0;
}
