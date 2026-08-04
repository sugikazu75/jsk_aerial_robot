# ROS2 build of gimbalrotor.
#
# Covers the three pluginlib libraries - robot model, controller, navigator -
# and the config and description files. That is the whole package: it has no
# nodes of its own.
#
# There is no ROS2 bringup for this robot yet, and it is not a launch-writing
# job. gimbalrotor only ever simulated in Gazebo (it ships no MuJoCo model) and
# it needs aerial_robot_model's servo_bridge for the gimbals, neither of which
# has been ported. See docs/ros2_migration.md.

add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=2)

find_package(ament_cmake REQUIRED)

find_package(aerial_robot_control REQUIRED)
find_package(aerial_robot_estimation REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(aerial_robot_ros_compat REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(Eigen3 REQUIRED)

set(GIMBALROTOR_DEPS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  aerial_robot_ros_compat
  geometry_msgs
  pluginlib
  rclcpp
  sensor_msgs
  spinal
  std_msgs
  tf2
)

include_directories(
  include
  # aerial_robot_control and aerial_robot_estimation export their targets, but
  # their headers are not always propagated onto consumers' compile lines.
  ${aerial_robot_control_INCLUDE_DIRS}
  ${aerial_robot_estimation_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(gimbalrotor_robot_model SHARED src/model/gimbalrotor_robot_model.cpp)
ament_target_dependencies(gimbalrotor_robot_model ${GIMBALROTOR_DEPS})
target_link_libraries(gimbalrotor_robot_model aerial_robot_model::aerial_robot_model_lib)

add_library(gimbalrotor_controller SHARED src/control/gimbalrotor_controller.cpp)
ament_target_dependencies(gimbalrotor_controller ${GIMBALROTOR_DEPS})
target_link_libraries(gimbalrotor_controller
  gimbalrotor_robot_model
  aerial_robot_control::flight_control_pluginlib)

add_library(gimbalrotor_navigation SHARED src/gimbalrotor_navigation.cpp)
ament_target_dependencies(gimbalrotor_navigation ${GIMBALROTOR_DEPS})
target_link_libraries(gimbalrotor_navigation aerial_robot_control::flight_navigation)

pluginlib_export_plugin_description_file(aerial_robot_model plugins/robot_model_plugins.ros2.xml)
pluginlib_export_plugin_description_file(aerial_robot_control plugins/control_plugins.ros2.xml)
pluginlib_export_plugin_description_file(aerial_robot_control plugins/navigation_plugin.ros2.xml)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS gimbalrotor_robot_model gimbalrotor_controller gimbalrotor_navigation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY config plugins robots urdf
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# Only the python launch file; the .launch XML beside it is the ROS1 one and
# starts gazebo, the spinal bridge and servo_bridge.
install(FILES launch/bringup.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${GIMBALROTOR_DEPS})

ament_package()
