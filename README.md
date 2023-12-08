# ros_robotic_tools

ROS wrapper for C++ library robotic_tools as a ROS Action Server/Client.

## Standalone
Example using the forwarding pipetting.
``` sh
$ roslaunch ros_robotic_tools pipette_tool_forward_pipetting.launch
```

## ROS Action Server/Client (High Level)
Uses the [PipetteCommand.msg](./msg/PipetteCommand.msg) to control the digital pipette which reasons about volume and real pipette techniques.

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

## ROS Action Server/Client (Low Level)
Uses the [PipetteCommandRaw.msg](./msg/PipetteCommandRaw.msg) to control the digital pipette which reasons about the pipette distance travel instead of volume. Usefull for calibration, when the relashionship between the volume and distance is not known.

Run the Action Server:
``` sh
$ roslaunch ros_robotic_tools pipette_tool_raw_action_server.launch device:=/dev/ttyACM0 baudrate:=9600
```

Run the Action Client examples:
``` sh
# Up and down motion
$ roslaunch ros_robotic_tools pipette_tool_raw_action_client_up_down.launch.launch

```
