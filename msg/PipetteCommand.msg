uint8 ACTION_HOMING=0               # Initialies the pipette
uint8 ACTION_MOVE_TO_SECOND_STOP=1  # Move to the 2nd STOP, lowest position before ejecting tip. If the volume parameter
                                    # is different from 0, then the pipette will move to the 1st STOP position (2nd Stop + upward offset wrt. the volume parameter).
uint8 ACTION_ASPIRATE=2             # Move upward wrt. the volume parameter
uint8 ACTION_ASPIRATE_SEQ=3         # Move upward and then downward times the sequence parameter, e.g. a sequence of 3 would do upward, downward and upward.
uint8 ACTION_DISPENSE=4             # Move downward wrt. the volume parameter
uint8 ACTION_DISPENSE_SEQ=5         # Move downward and then upward times the sequence parameter, e.g. a sequence of 3 would do downward, upward and downward.
uint8 ACTION_EJECT=6                # Move to the lowest position, then move upward to the 2nd stop.

uint8 action

float64 velocity        # Max velocity (m/s)
uint32  volume          # Volume to aspirate/dispense (nL)
uint32  sequence        # Number of back and forth (only used in actions ending with '_SEQ')
