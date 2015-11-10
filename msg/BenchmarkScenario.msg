# Types of benchmarks
uint8 NONE = 0 # No benchmark running
uint8 TBM  = 1 # Task benchmark
uint8 FBM  = 2 # Functionality benchmark

# e.g. TASK_BENCHMARK 1 "TBM 1: Assembly aid tray"
std_msgs/UInt64 type
std_msgs/UInt32 type_id
std_msgs/String description