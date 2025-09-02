# ros_robotic_tools

ROS wrapper for C++ library robotic_tools as a ROS Action Server/Client.

## ROS Action Server/Client (High Level)
Uses the [PipetteCommand.msg](./msg/PipetteCommand.msg) to control the digital pipette which reasons about volume and real pipette techniques.

Run the Action Server:
``` sh
$ roslaunch ros_robotic_tools action_server.launch device:=/dev/ttyACM0 baudrate:=9600
```

Run primitive commands from the Action Client examples:
``` sh
# Aspirate 10 uL @ 0.01 m/s
$ roslaunch ros_robotic_tools aspirate.launch volume:=10000 speed:=0.01

# Dispense 1 uL @ 0.03 m/s
$ roslaunch ros_robotic_tools aspirate.launch volume:=1000 speed:=0.03

# Move to 1st Stop (offset wrt. the 2nd Stop position)
roslaunch ros_robotic_tools move_to_1st_stop.launch upward_volume_offset:=10000 speed:=0.01

# Move to 2nd Stop
roslaunch ros_robotic_tools move_to_2nd_stop.launch speed:=0.01

# Eject the Tip
roslaunch ros_robotic_tools eject_tip
```

Run pipetting techniques from the Action Client examples:
``` sh
# Forward Pipetting
$ roslaunch ros_robotic_tools forward_pipetting.launch

# Reverse Pipetting
$ roslaunch ros_robotic_tools reverse_pipetting.launch
```

## ROS Action Server/Client (Low Level)
Uses the [PipetteCommandRaw.msg](./msg/PipetteCommandRaw.msg) to control the digital pipette which reasons about the pipette distance travel instead of volume. Usefull for calibration, when the relashionship between the volume and distance is not known.

Run the Action Server:
``` sh
$ roslaunch ros_robotic_tools raw_action_server.launch device:=/dev/ttyACM0 baudrate:=9600
```

Run the Action Client examples:
``` sh
# Up and down motion
$ roslaunch ros_robotic_tools raw_up_down.launch

```
