cmake_minimum_required(VERSION 3.0.2)
project(flying_humanoid)

find_package(catkin REQUIRED COMPONENTS
  mujoco_ros_utils
  )

catkin_package()

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
