cmake_minimum_required(VERSION 3.0.2)
project(gimbalrotor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  aerial_motion_planning
  mujoco_ros_utils
  spinal
  pluginlib
  roscpp
  )

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)  # the whole-body controller
find_package(proxsuite REQUIRED)

# python: src/gimbalrotor (GimbalrotorInterface)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS aerial_motion_planning aerial_robot_control aerial_robot_estimation aerial_robot_model aerial_robot_msgs pluginlib roscpp spinal
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(gimbalrotor_controller src/control/gimbalrotor_controller.cpp)
target_link_libraries(gimbalrotor_controller ${catkin_LIBRARIES})

add_library(gimbalrotor_whole_body_controller src/control/gimbalrotor_whole_body_controller.cpp)
target_link_libraries(gimbalrotor_whole_body_controller gimbalrotor_controller ${catkin_LIBRARIES} pinocchio::pinocchio proxsuite::proxsuite)

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

catkin_install_python(PROGRAMS
  scripts/planning/plan_perching_motion.py
  scripts/visualization/trajectory_playback.py
  scripts/visualization/contact_wrench_visualizer.py
  scripts/visualization/display_pose.py
  scripts/execution/follow_trajectory.py
  scripts/execution/perching_task.py
  scripts/execution/check_pre_hook_motion.py
  scripts/execution/hanging_teleop.py
  scripts/execution/hand_accuracy_test.py
  scripts/analysis/compare_hand_accuracy.py
  scripts/analysis/benchmark_perching.py
  scripts/analysis/plot_perching_experiment.py
  scripts/simulation/make_perching_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Keep this package's model generation and joint armature settings local to gimbalrotor.
file(GLOB_RECURSE GIMBALROTOR_MUJOCO_SOURCES CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/urdf/*
  ${PROJECT_SOURCE_DIR}/robots/*)
set(GIMBALROTOR_MUJOCO_STAMP ${CMAKE_CURRENT_BINARY_DIR}/gimbalrotor_mujoco_model.stamp)
add_custom_command(
  OUTPUT ${GIMBALROTOR_MUJOCO_STAMP}
  COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/simulation/generate_mujoco_model.py
          ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml
  COMMAND ${CMAKE_COMMAND} -E touch ${GIMBALROTOR_MUJOCO_STAMP}
  DEPENDS ${PROJECT_SOURCE_DIR}/scripts/simulation/generate_mujoco_model.py
          ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml
          ${GIMBALROTOR_MUJOCO_SOURCES}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMENT "Generating the gimbalrotor MuJoCo model")
add_custom_target(generate_mujoco_model ALL DEPENDS ${GIMBALROTOR_MUJOCO_STAMP})

# the MuJoCo world of the hanging-perching demo from the configuration of the planner (the scene, the
# task frame, the start and the folded arms), again whenever it, the robot or the script changes
file(GLOB_RECURSE PERCHING_WORLD_ROBOT CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/urdf/quad_with_dual_arm/*
  ${PROJECT_SOURCE_DIR}/robots/quad_with_dual_arm/*)
add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/worlds/quad_with_dual_arm_perching.xml
         ${PROJECT_SOURCE_DIR}/worlds/quad_with_dual_arm_perching.yaml
  COMMAND ${PYTHON_EXECUTABLE} ${PROJECT_SOURCE_DIR}/scripts/simulation/make_perching_world.py
          --config ${PROJECT_SOURCE_DIR}/config/quad_with_dual_arm/perching_motion.yaml
          --name quad_with_dual_arm_perching
  DEPENDS ${PROJECT_SOURCE_DIR}/scripts/simulation/make_perching_world.py
          ${PROJECT_SOURCE_DIR}/config/quad_with_dual_arm/perching_motion.yaml
          ${PERCHING_WORLD_ROBOT}
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  COMMENT "Generating the MuJoCo world quad_with_dual_arm_perching")
add_custom_target(generate_perching_world ALL
  DEPENDS ${PROJECT_SOURCE_DIR}/worlds/quad_with_dual_arm_perching.xml)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_subdirectory(test)
endif()
