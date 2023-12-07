# ros_robotic_tools

ROS wrapper for C++ library robotic_tools as a ROS Action Server.

## Standalone
Example using the forwarding pipetting.
``` sh
$ roslaunch ros_robotic_tools pipette_tool_forward_pipetting.launch
```

## ROS Action Server/Client
Run the Action Server:
``` sh
$ roslaunch ros_robotic_tools pipette_tool_action_server.launch device:=/dev/ttyACM0 baudrate:=9600
```

Run the Action Client examples:
``` sh
# Forward Pipetting
$ roslaunch ros_robotic_tools pipette_tool_action_client_forward_pipetting.launch.launch

# Reverse Pipetting
$ roslaunch ros_robotic_tools pipette_tool_action_client_forward_pipetting.launch.launch
```
