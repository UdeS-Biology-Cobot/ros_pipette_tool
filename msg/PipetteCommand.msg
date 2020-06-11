uint8 MODE_FORWARD=0
uint8 MODE_REVERSE=1
uint8 mode

uint8 ACTION_INIT=0
uint8 ACTION_ASPIRATE=1
uint8 ACTION_DISPENSE=2
uint8 ACTION_EJECT=3
uint8 action

float64 velocity  # Max velocity
uint32  volume_nl # Volume to aspirate/dispense in nano liters
uint32  offset_nl # FORWARD: Used to complety dispense the volume (DISPENSE)
                  # REVERSE: Add the offset to the volume (ASPIRATE)
