# A description of the object.
at_work_robot_example_ros/ObjectIdentifier object

# When the object describes a class (i.e. the instance_id of the label is
# *not* set) the quantity of the object must be specified. For object
# instances (i.e. the instance_id of the label *is* set) the quantity is
# ignored.
std_msgs/UInt64 quantity

# Either the container or the location field must be set
# The container in which this item is stored
at_work_robot_example_ros/ObjectIdentifier container

# The location at which this item is stored
at_work_robot_example_ros/LocationIdentifier location