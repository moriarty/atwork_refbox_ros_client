cmake_minimum_required(VERSION 2.8.7)
project(at_work_robot_example_ros)


find_package(
  Boost REQUIRED COMPONENTS system filesystem thread)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    protobuf_comm
    raw_msgs
    at_work_central_factory_hub_ros_msgs
    std_msgs
    geometry_msgs
    message_generation
)


SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -DHAVE_LIBCRYPTO")

include_directories(
  ros/include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    protobuf_comm
    raw_msgs
    at_work_central_factory_hub_ros_msgs
)

add_executable(robot_example_ros
   ros/src/robot_example_ros_node.cpp
   ros/src/robot_example_ros.cpp
)

target_link_libraries(robot_example_ros
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

install(
  DIRECTORY ros/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  TARGETS robot_example_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


