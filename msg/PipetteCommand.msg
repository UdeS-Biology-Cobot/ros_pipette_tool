uint8 MODE_FORWARD=0
uint8 MODE_REVERSE=1
uint8 mode

uint8 ACTION_HOMING=0
uint8 ACTION_INIT=1
uint8 ACTION_ASPIRATE=2
uint8 ACTION_ASPIRATE_SEQ=3
uint8 ACTION_DISPENSE=4
uint8 ACTION_DISPENSE_SEQ=5
uint8 ACTION_EJECT=6
uint8 action

float64 velocity  # Max velocity
uint32  volume_nl # Volume to aspirate/dispense in nano liters
uint32  offset_nl # FORWARD: Used to complety dispense the volume (DISPENSE)
                  # REVERSE: Add the offset to the volume (ASPIRATE)
uint32  sequence  # Sequence used for ACTION_ASPIRATE_SEQ & ACTION_DISPENSE_SEQ
