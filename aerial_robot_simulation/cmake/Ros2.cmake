add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=2)

find_package(ament_cmake REQUIRED)

find_package(aerial_robot_estimation REQUIRED)
find_package(aerial_robot_ros_compat REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(tf2 REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(mujoco_ros REQUIRED)
find_package(mujoco_ros_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(AERIAL_ROBOT_SIMULATION_DEPS
  aerial_robot_estimation
  aerial_robot_ros_compat
  geometry_msgs
  hardware_interface
  nav_msgs
  sensor_msgs
  spinal
  tf2
  lifecycle_msgs
  mujoco_ros
  mujoco_ros_control
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

include_directories(
  include
  # aerial_robot_estimation exports its targets but not its own header directory
  # on their interface includes; see docs/ros2_migration.md.
  ${aerial_robot_estimation_INCLUDE_DIRS})

add_library(mujoco_thrust_visualizer SHARED
  src/mujoco/thrust_visualizer_plugin.cpp)
ament_target_dependencies(mujoco_thrust_visualizer ${AERIAL_ROBOT_SIMULATION_DEPS})

# The simulated flight controller plus the ros2_control hardware component that
# drives it. Kept in one library: the estimator and the control core are folded
# into the component, since ros2_control cannot pass an object to a controller.
add_library(aerial_robot_mujoco_system SHARED
  src/mujoco/aerial_robot_spinal.cpp
  src/mujoco/aerial_robot_mujoco_system.cpp)
ament_target_dependencies(aerial_robot_mujoco_system ${AERIAL_ROBOT_SIMULATION_DEPS})
target_link_libraries(aerial_robot_mujoco_system spinal::spinal_flight_controller spinal::spinal_math)

pluginlib_export_plugin_description_file(mujoco_ros mujoco_visualization_plugin.ros2.xml)
pluginlib_export_plugin_description_file(mujoco_ros_control aerial_robot_mujoco_system_plugin.ros2.xml)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS mujoco_thrust_visualizer aerial_robot_mujoco_system
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES
  mujoco_visualization_plugin.ros2.xml
  aerial_robot_mujoco_system_plugin.ros2.xml
  DESTINATION share/${PROJECT_NAME})

# Mujoco.yaml is read by mujoco.launch.py, which lifts its `simulation:` block
# onto the hardware component's node. The ros_control entries in the same file
# are ROS1-only and ignored.
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# Only the *.launch.py files: the ROS1 .launch files here reference gazebo_ros
# and the controller_manager spawner, neither of which exists under ROS2.
install(FILES
  launch/mujoco.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

# Likewise only the ros2_control description macro; spinal.gazebo.xacro emits
# <gazebo> tags that mean nothing here.
install(FILES
  xacro/spinal.ros2_control.xacro
  DESTINATION share/${PROJECT_NAME}/xacro)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AERIAL_ROBOT_SIMULATION_DEPS})

ament_package()
