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
)

## Generate actions in the 'action' folder
add_action_files(
   FILES
   PipetteCommand.action
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

add_executable(pipette_tool_example src/example.cpp)
add_executable(pipette_tool_action_server src/pipette_tool_action_server.cpp)
add_executable(pipette_tool_action_client src/pipette_tool_action_client.cpp)
add_executable(pipette_tool_action_client_multi_dispense src/pipette_tool_action_client_multi_dispense.cpp)

target_link_libraries(pipette_tool_example
   ${catkin_LIBRARIES}
   robotic_tool
)

target_link_libraries(pipette_tool_action_server
   ${catkin_LIBRARIES}
   robotic_tool
   )

target_link_libraries(pipette_tool_action_client
   ${catkin_LIBRARIES}
)

target_link_libraries(pipette_tool_action_client_multi_dispense
   ${catkin_LIBRARIES}
)

add_dependencies(pipette_tool_action_server ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_action_client ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(pipette_tool_action_client_multi_dispense ${PROJECT_NAME}_generate_messages_cpp)


install(TARGETS
    pipette_tool_example
    pipette_tool_action_server
    pipette_tool_action_client
    pipette_tool_action_client_multi_dispense
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
