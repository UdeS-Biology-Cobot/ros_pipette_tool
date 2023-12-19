cmake_minimum_required(VERSION 3.12)
project(ros_robotic_tools)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PipetteCommand.msg
   PipetteCommandRaw.msg
)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   PipetteCommand.action
   PipetteCommandRaw.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

# Robotic Tool Lib submodule includes + sources
set(ROBOTIC_TOOL_INCLUDE_DIRS ext/pipette-tool-sw/include)
file(GLOB_RECURSE ROBOTIC_TOOL_SRC CONFIGURE_DEPENDS "ext/pipette-tool-sw/src/*.cpp")

catkin_package(
  # LIBRARIES
  #   ${PROJECT_NAME}
  INCLUDE_DIRS
    include
    ${ROBOTIC_TOOL_INCLUDE_DIRS}
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    actionlib_msgs
    message_runtime
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${ROBOTIC_TOOL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(robotic_tool ${ROBOTIC_TOOL_SRC})

add_executable(pipette_tool_forward_pipetting src/pipette_tool_forward_pipetting.cpp)

add_executable(pipette_tool_action_server src/pipette_tool_action_server.cpp)
add_executable(pipette_tool_action_client_forward_pipetting src/pipette_tool_action_client_forward_pipetting.cpp)
add_executable(pipette_tool_action_client_reverse_pipetting src/pipette_tool_action_client_reverse_pipetting.cpp)
add_executable(pipette_tool_action_client_aspirate src/pipette_tool_action_client_aspirate.cpp)
add_executable(pipette_tool_action_client_dispense src/pipette_tool_action_client_dispense.cpp)
add_executable(pipette_tool_action_client_move_to_1st_stop src/pipette_tool_action_client_move_to_1st_stop.cpp)
add_executable(pipette_tool_action_client_move_to_2nd_stop src/pipette_tool_action_client_move_to_2nd_stop.cpp)
add_executable(pipette_tool_action_client_eject_tip src/pipette_tool_action_client_eject_tip.cpp)

add_executable(pipette_tool_raw_action_server src/pipette_tool_raw_action_server.cpp)
add_executable(pipette_tool_raw_action_client_up_down src/pipette_tool_raw_action_client_up_down.cpp)

target_link_libraries(pipette_tool_forward_pipetting
   ${catkin_LIBRARIES}
   robotic_tool
)

target_link_libraries(pipette_tool_action_server
   ${catkin_LIBRARIES}
   robotic_tool
)

target_link_libraries(pipette_tool_action_client_forward_pipetting
   ${catkin_LIBRARIES}
)

target_link_libraries(pipette_tool_action_client_reverse_pipetting
   ${catkin_LIBRARIES}
)

target_link_libraries(pipette_tool_action_client_aspirate
   ${catkin_LIBRARIES}
)

target_link_libraries(pipette_tool_action_client_dispense
   ${catkin_LIBRARIES}
)

target_link_libraries(pipette_tool_action_client_move_to_1st_stop
   ${catkin_LIBRARIES}
)

target_link_libraries(pipette_tool_action_client_move_to_2nd_stop
   ${catkin_LIBRARIES}
)

target_link_libraries(pipette_tool_action_client_eject_tip
   ${catkin_LIBRARIES}
)

target_link_libraries(pipette_tool_raw_action_server
   ${catkin_LIBRARIES}
   robotic_tool
)

target_link_libraries(pipette_tool_raw_action_client_up_down
   ${catkin_LIBRARIES}
)

add_dependencies(pipette_tool_action_server ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_action_client_forward_pipetting ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_action_client_reverse_pipetting ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_action_client_aspirate ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_action_client_dispense ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_action_client_move_to_1st_stop ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_action_client_move_to_2nd_stop ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_action_client_eject_tip ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(pipette_tool_raw_action_server ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_raw_action_client_up_down ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS
    pipette_tool_forward_pipetting
    pipette_tool_action_server
    pipette_tool_action_client_forward_pipetting
    pipette_tool_action_client_reverse_pipetting
    pipette_tool_action_client_aspirate
    pipette_tool_action_client_dispense
    pipette_tool_action_client_move_to_1st_stop
    pipette_tool_action_client_move_to_2nd_stop
    pipette_tool_action_client_eject_tip
    pipette_tool_raw_action_server
    pipette_tool_raw_action_client_up_down
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
