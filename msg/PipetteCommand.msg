uint8 ACTION_MOVE_TO_1ST_STOP=0     # Move to the 1st Stop position, upward offset (volume parameter) wrt. the 2nd Stop position
uint8 ACTION_MOVE_TO_2ND_STOP=1     # Move to the 2nd Stop position, lowest position before ejecting tip
uint8 ACTION_ASPIRATE=2             # Move upward wrt. the volume parameter
uint8 ACTION_DISPENSE=3             # Move downward wrt. the volume parameter
uint8 ACTION_EJECT=4                # Move to the lowest position, then move upward to the 2nd stop.

uint8 action

float64 velocity        # Max velocity (m/s)
float64 volume          # Volume to aspirate/dispense (Liter)
uint32  back_and_forth  # Number of back and forth, only used in ASPIRATE and DISPENSE actions
                        # E.g. ACTION_ASPIRATE with back_and_forth = 3 would do this sequence: aspirate, dispense, aspirate
