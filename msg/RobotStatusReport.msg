# Robot Capabilities
uint8 NAVIGATION = 0 # Navigation (e.g. planning or execution)
uint8 MANIPULATION = 1 # Manipulation (e.g. grasp planning/execution, arm motion planning/execution)
uint8 PERCEPTION = 2 # Perception (e.g. RGB or depth image processing)
uint8 TASK = 3 # Task planning or task execution

# The currently active capability
std_msgs/UInt32 capability

# The functionality which provides the above capability (e.g. a software component)
std_msgs/String functionality

# Arbitrary information to further describe the capability (e.g. error codes or detected objects).
std_msgs/String meta_data
