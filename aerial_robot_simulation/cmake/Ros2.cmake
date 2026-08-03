add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=2)

find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(mujoco_ros REQUIRED)
find_package(mujoco_ros_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

set(AERIAL_ROBOT_SIMULATION_DEPS
  hardware_interface
  lifecycle_msgs
  mujoco_ros
  mujoco_ros_control
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

include_directories(include)

add_library(mujoco_thrust_visualizer SHARED
  src/mujoco/thrust_visualizer_plugin.cpp)
ament_target_dependencies(mujoco_thrust_visualizer ${AERIAL_ROBOT_SIMULATION_DEPS})

add_library(aerial_robot_mujoco_system SHARED
  src/mujoco/aerial_robot_mujoco_system.cpp)
ament_target_dependencies(aerial_robot_mujoco_system ${AERIAL_ROBOT_SIMULATION_DEPS})

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

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AERIAL_ROBOT_SIMULATION_DEPS})

ament_package()
