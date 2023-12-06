#include <ros/ros.h>
//#include <chrono>
//#include <cstdlib>
#include <string>
//#include <thread>


#include <robotic_tool/pipette_tool/protocol/posix/pt_master_protocol.h>
#include <robotic_tool/pipette_tool/posix/pipette_tool_control.h>

#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

void forward(PipetteToolControl pt_ctrl, uint32_t nl, uint32_t offset_nl, double speed, PipetteToolControlErr* p_err)
{

	pt_ctrl.forward_init(offset_nl, speed, p_err);
	if (pt_ctrl.err_get(p_err)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
		return;
	}

	pt_ctrl.forward_aspirate(nl, speed, p_err);
	if (pt_ctrl.err_get(p_err)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
		return;
	}

	pt_ctrl.forward_dispense(nl, offset_nl, speed, p_err);
	if (pt_ctrl.err_get(p_err)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
		return;
	}
}

void reverse(PipetteToolControl pt_ctrl, uint32_t nl, uint32_t offset_nl, double speed, PipetteToolControlErr* p_err)
{

	pt_ctrl.reverse_init(speed, p_err);
	if (pt_ctrl.err_get(p_err)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
		return;
	}

	pt_ctrl.reverse_aspirate(nl, offset_nl, speed, p_err);
	if (pt_ctrl.err_get(p_err)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
		return;
	}

	pt_ctrl.reverse_dispense(nl, speed, p_err);
	if (pt_ctrl.err_get(p_err)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
		return;
	}
}


void eject(PipetteToolControl pt_ctrl, PipetteToolControlErr* p_err) {

	pt_ctrl.eject(p_err);
	if (pt_ctrl.err_get(p_err)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)*p_err << std::endl;
		return;
	}
}








int main(int argc, char **argv)
{
	ros::init(argc, argv, "pipette_tool_example");


	boost::asio::io_service io;
	boost::asio::serial_port port(io);

	const std::string device = "/dev/pipette_tool";
	uint32_t baudrate = 2000000;


	PtProtocolErr err_pt = PtProtocolErr::NONE;
	PipetteToolControlErr err_pt_ctrl = PipetteToolControlErr::NONE;
	PtMasterProtocol pt = PtMasterProtocol(&port, baudrate, device);




	PipetteToolControl pt_ctrl(&pt, &err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}

	uint32_t curr_vol;



	uint32_t max_vol = pt_ctrl.get_max_volume_nl(&err_pt_ctrl);
	if (pt_ctrl.err_get(&err_pt_ctrl)) {
		std::cout << "PipetteToolControl error code = " << (uint32_t)err_pt_ctrl << std::endl;
		return 0;
	}
	std::cout << "max_vol = " << max_vol << std::endl;




	uint32_t nl = 75000;
	uint32_t offset_nl = 35000;
	double speed = 0.02;





	//forward(pt_ctrl, nl, offset_nl, speed, &err_pt_ctrl);

	//reverse(pt_ctrl, nl, offset_nl, speed, &err_pt_ctrl);

	eject(pt_ctrl, &err_pt_ctrl);


  ros::spin();

  ROS_INFO("ROS stopping, shutting down pipelines");

  return EXIT_SUCCESS;
}
