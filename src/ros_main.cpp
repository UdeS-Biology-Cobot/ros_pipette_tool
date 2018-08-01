#include <ros/ros.h>
//#include <chrono>
//#include <cstdlib>
#include <string>
//#include <thread>


#include <robotic_tools/error_handle/error_handle.h>
#include <robotic_tools/pipette_tool/posix/pipette_tool_serial_master.h>
#include <robotic_tools/pipette_tool/protocol/pipette_tool_master_protocol.h>


ErrorHandle err_handle = ErrorHandle();
ErrorHandle* p_err = &err_handle;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robotic_tools");


  PipetteToolBaseProtocol a = PipetteToolBaseProtocol(0x22);
  

  //std::string s = std::to_string(p_err->err_code_offset);
  ROS_INFO("%d", a.MAX_REGISTERS);

  ROS_INFO("Init");


  ros::spin();

  ROS_INFO("ROS stopping, shutting down pipelines");

  return EXIT_SUCCESS;
}
