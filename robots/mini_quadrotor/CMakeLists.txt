cmake_minimum_required(VERSION 3.0.2)
project(mini_quadrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  mujoco_ros_control
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_oc
  ocs2_pinocchio_interface
  ocs2_robotic_tools
  pluginlib
  )

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

catkin_package(
  LIBRARIES mini_quadrotor_controllib
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model ocs2_core ocs2_ddp ocs2_mpc ocs2_oc ocs2_pinocchio_interface ocs2_robotic_tools pluginlib
  )

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(mini_quadrotor_controllib src/control/ocp.cpp)
target_link_libraries(mini_quadrotor_controllib ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS mini_quadrotor_controllib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config launch plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY bin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
