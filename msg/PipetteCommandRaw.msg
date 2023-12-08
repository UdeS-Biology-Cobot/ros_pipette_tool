uint8 ACTION_HOMING=0           # Initialies the pipette, no parameters
uint8 ACTION_MOVE_TO_TOP=1      # Move to the highest position (upwards)
uint8 ACTION_MOVE_TO_TIP=2      # Move to the position near the tip head
uint8 ACTION_MOVE_TO_BOTTOM=3   # Move to the lowest position (downwards)

uint8 ACTION_MOVE_UPWARD=4      # Move upwards by a number of mm (relative to current position)
uint8 ACTION_MOVE_DOWNWARD=5    # Move downwards by a number of mm (relative to current position)

uint8 action

float64 velocity        # Max velocity (m/s)
uint32  position        # Distance to travel (mm). Only used with ACTION_MOVE_UPWARD/DOWNWARD.
