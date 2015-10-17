# The object class (e.g. "EM-01") consists of the type ("EM") and a type ID (1).
# The type of the object
std_msgs/UInt64 type

# The type ID of the object
std_msgs/UInt64 type_id

# All labeled objects also have an instance ID. For example, there could
# exist two instances of EM-01 containers, called "EM-01-001" and
# "EM-01-002" where "EM-01" represents the item's class and "001"/"002" are
# the instance IDs.
std_msgs/UInt64 instance_id

# A human-readable description of the identifier
std_msgs/String description