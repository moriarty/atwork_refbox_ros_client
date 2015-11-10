# positions of the drill bit
uint8 AT_TOP      = 0  # The drill bit is in the topmost position
uint8 AT_BOTTOM   = 1  # The drill bit is in the lowermost position
uint8 MOVING_UP   = 2  # The drill bit is moving upwards
uint8 MOVING_DOWN = 3  # The drill bit is moving downwards
uint8 UNKNOWN     = 4  # The position of the drill bit cannot be determined (e.g. because there is no connection to the drilling machine)

std_msgs/UInt64 state