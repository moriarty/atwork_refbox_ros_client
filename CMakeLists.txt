cmake_minimum_required(VERSION 2.8.7)
project(at_work_robot_example_ros)

## if: using catkin_make
if(DEFINED _CATKIN_ALL_INCLUDED_)
  find_package(catkin REQUIRED
    COMPONENTS
      roscpp
      protobuf_comm
      rockin_msgs
      std_msgs
      geometry_msgs
      message_generation
  )

  find_package(Boost REQUIRED COMPONENTS system filesystem thread)

  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -DHAVE_LIBCRYPTO")

  include_directories(
    ros/include
    ${catkin_INCLUDE_DIRS}
  )

  add_message_files(FILES
    AttentionMessage.msg
    BenchmarkState.msg
    BenchmarkFeedback.msg
    DrillingMachineCommand.msg
    DrillingMachineStatus.msg
    Inventory.msg
    Item.msg
    LocationIdentifier.msg
    ObjectIdentifier.msg
    Order.msg
    OrderInfo.msg
    TriggeredConveyorBeltStatus.msg
    TriggeredConveyorBeltCommand.msg
    BenchmarkScenario.msg
  )

  generate_messages(
    DEPENDENCIES
      std_msgs
      geometry_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      roscpp
      protobuf_comm
      rockin_msgs
      message_runtime
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
    TARGETS robot_example_ros
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

## else: using cmake
else()

endif()
