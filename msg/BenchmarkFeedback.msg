# phases of benchmarks
uint8 EXECUTION   = 0
uint8 CALIBRATION = 1
uint8 PREPARATION = 2
# Current benchmark phase to terminate
std_msgs/UInt64 phase_to_terminate

# FBM1
std_msgs/String object_class_name
geometry_msgs/Pose object_pose

# FBM1+FMB2
std_msgs/String object_instance_name

# FBM2
std_msgs/Bool grasp_notification
geometry_msgs/Pose end_effector_pose

# TBM1
std_msgs/String assembly_aid_tray_id
std_msgs/String container_id

uint8 UNUSABLE = 1
uint8 FAULTY   = 2
uint8 PERFECT  = 3
# TBM2 plate_state
std_msgs/UInt64 plate_state_after_receiving
std_msgs/UInt64 plate_state_after_drilling




