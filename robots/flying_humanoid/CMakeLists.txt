cmake_minimum_required(VERSION 3.0.2)
project(flying_humanoid)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  mujoco_ros_utils
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  spinal
  std_msgs
  std_srvs
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(crocoddyl REQUIRED)
find_package(urdfdom REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  )

add_library(flying_humanoid_robot_model src/robot_model.cpp)
target_link_libraries(flying_humanoid_robot_model ${catkin_LIBRARIES} pinocchio::pinocchio ${urdfdom_LIBRARIES})

add_executable(walk_planner src/walk_planner.cpp src/walk_planner_node.cpp)
target_link_libraries(walk_planner flying_humanoid_robot_model ${catkin_LIBRARIES} crocoddyl::crocoddyl)

install(TARGETS flying_humanoid_robot_model walk_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY config launch meshes robots urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

catkin_install_python(PROGRAMS scripts/load_initial_joint_states.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

mujoco_model_convert(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml)
