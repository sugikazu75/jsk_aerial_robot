cmake_minimum_required(VERSION 3.0.2)
project(gundam)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  aerial_robot_model
  aerial_robot_msgs
  aerial_robot_simulation
  aerial_robot_control
  dragon
  roscpp
  sensor_msgs
  pluginlib
  spinal
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_model aerial_robot_msgs pluginlib roscpp
)

