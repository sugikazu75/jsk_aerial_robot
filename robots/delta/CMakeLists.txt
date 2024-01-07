cmake_minimum_required(VERSION 3.0.2)
project(delta)

set(CMAKE_CXX_STANDARD 17)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  casadi_eigen
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(casadi REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs casadi_eigen pluginlib roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIRS}
  )

add_library(delta_controller src/control/delta_controller.cpp)
target_link_libraries(delta_controller ${catkin_LIBRARIES} casadi)

add_executable(delta_model src/model/delta_model.cpp)
target_link_libraries(delta_model ${catkin_LIBRARIES} ${pinocchio_LIBRARIES})
