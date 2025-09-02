cmake_minimum_required(VERSION 3.12)
project(ros_pipette_tool)

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

add_executable(action_server src/action_server.cpp)
add_executable(forward_pipetting src/forward_pipetting.cpp)
add_executable(reverse_pipetting src/reverse_pipetting.cpp)
add_executable(aspirate src/aspirate.cpp)
add_executable(dispense src/dispense.cpp)
add_executable(move_to_1st_stop src/move_to_1st_stop.cpp)
add_executable(move_to_2nd_stop src/move_to_2nd_stop.cpp)
add_executable(eject_tip src/eject_tip.cpp)

add_executable(raw_action_server src/raw_action_server.cpp)
add_executable(raw_up_down src/raw_up_down.cpp)


target_link_libraries(action_server ${catkin_LIBRARIES} robotic_tool)
target_link_libraries(forward_pipetting ${catkin_LIBRARIES})
target_link_libraries(reverse_pipetting ${catkin_LIBRARIES})
target_link_libraries(aspirate ${catkin_LIBRARIES})
target_link_libraries(dispense ${catkin_LIBRARIES})
target_link_libraries(move_to_1st_stop ${catkin_LIBRARIES})
target_link_libraries(move_to_2nd_stop ${catkin_LIBRARIES})
target_link_libraries(eject_tip ${catkin_LIBRARIES})

target_link_libraries(raw_action_server ${catkin_LIBRARIES} robotic_tool)
target_link_libraries(raw_up_down ${catkin_LIBRARIES})

add_dependencies(action_server ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(forward_pipetting ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(reverse_pipetting ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(aspirate ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(dispense ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(move_to_1st_stop ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(move_to_2nd_stop ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(eject_tip ${PROJECT_NAME}_generate_messages_cpp)

add_dependencies(raw_action_server ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(raw_up_down ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS
    action_server
    forward_pipetting
    reverse_pipetting
    aspirate
    dispense
    move_to_1st_stop
    move_to_2nd_stop
    eject_tip
    raw_action_server
    raw_up_down
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
