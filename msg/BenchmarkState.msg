# Time since benchmark start
std_msgs/Time benchmark_time

# Current benchmark state
std_msgs/UInt64  state

# Current benchmark phase
std_msgs/UInt64  phase

# Current benchmark scenario
BenchmarkScenario scenario

# Name of all known teams
std_msgs/String[] known_teams

# Name of teams which are currently connected to the RefBox
std_msgs/String[] connected_teams