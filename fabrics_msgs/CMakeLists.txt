cmake_minimum_required(VERSION 3.0.2)
project(fabrics_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  FabricsConstraintsGoal.msg
  FabricsJointSpaceGoal.msg
  FabricsObstacle.msg
  FabricsObstacleArray.msg
  FabricsPoseGoal.msg
  FabricsState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  std_msgs
)
