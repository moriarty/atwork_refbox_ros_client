# A unique identifier of this order
std_msgs/UInt64 id

# types of status
uint8 OFFERED     =  1  # The order is offered by the RefBox and can be processed by a team/robot
uint8 TIMEOUT     =  2  # The order has not been accepted in time or processing took too long
uint8 IN_PROGRESS =  3  # A team/robot is currently processing this order
uint8 PAUSED      =  4  # Temporarily pause processing this order
uint8 ABORTED     =  5  # Stop working on this order
uint8 FINISHED    =  6  # The order has been processed successfully
# The status of this order as specified in the Status enumeration
std_msgs/UInt64 status

# The requested product represented by their identifier
at_work_robot_example_ros/ObjectIdentifier object

# There are two different interpretations of this field:
# * When the destination field is set, the object(s) must be delivered to
#   this destination INSIDE of the container.
# * When the destination field is not set, the container represents the
#   location into which the object(s) must be delivered.
at_work_robot_example_ros/ObjectIdentifier container

# The already delivered quantity of the object
std_msgs/UInt64 quantity_delivered

# The requested quantity of the object. This is only required for object
# classes. When a concrete object instance is requested, this field is not
# set.
# If the objects are requested in a container (i.e. the container field is
# set), this field specifies the amount of objects inside the container!
std_msgs/UInt64 quantity_requested

# The location to where the product should be delivered. When the destination
# is not specified, the container must be set. This container the identifies
# the delivery location.
at_work_robot_example_ros/LocationIdentifier destination

# The location to where the product should be fetched
at_work_robot_example_ros/LocationIdentifier source

# The team which is currently processing this order
std_msgs/String processing_team