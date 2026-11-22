cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  mujoco_ros_utils
  spinal
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp spinal
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller src/control/gimbalrotor_controller.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_robot_model src/model/gimbalrotor_robot_model.cpp)
target_link_libraries(gimbalrotor_robot_model ${catkin_LIBRARIES})

add_library(gimbalrotor_navigation src/gimbalrotor_navigation.cpp)
target_link_libraries(gimbalrotor_navigation ${catkin_LIBRARIES})

install(DIRECTORY config plugins launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

catkin_install_python(PROGRAMS
  scripts/gimbal_test.py
  scripts/gimbal_test_times.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Keep this package's model generation and joint armature settings local to gimbalrotor.
file(GLOB_RECURSE GIMBALROTOR_MUJOCO_SOURCES CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/urdf/*
  ${PROJECT_SOURCE_DIR}/robots/*)
set(GIMBALROTOR_MUJOCO_STAMP ${CMAKE_CURRENT_BINARY_DIR}/gimbalrotor_mujoco_model.stamp)
add_custom_command(
  OUTPUT ${GIMBALROTOR_MUJOCO_STAMP}
  COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/generate_mujoco_model.py
          ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml
  COMMAND ${CMAKE_COMMAND} -E touch ${GIMBALROTOR_MUJOCO_STAMP}
  DEPENDS ${PROJECT_SOURCE_DIR}/scripts/generate_mujoco_model.py
          ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml
          ${GIMBALROTOR_MUJOCO_SOURCES}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMENT "Generating the gimbalrotor MuJoCo model")
add_custom_target(generate_mujoco_model ALL DEPENDS ${GIMBALROTOR_MUJOCO_STAMP})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
