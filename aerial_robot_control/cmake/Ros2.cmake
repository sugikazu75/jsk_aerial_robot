# ROS2 build of aerial_robot_control.
#
# Scope intentionally matches mini_quadrotor MuJoCo hovering:
#   - navigation: src/flight_navigation.cpp (+ joy parser)
#   - controllers: pose_linear / under_actuated / under_actuated_lqi /
#                  under_actuated_tilted_lqi / fully_actuated
#   - trajectory library used by navigation
#
# Excluded from ROS2 build for now:
#   - src/trajectory/test.cpp (ROS1 test node)
#   - dynamic_reconfigure generation itself (no ROS2 equivalent)

add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=2)

find_package(ament_cmake REQUIRED)

find_package(aerial_robot_estimation REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(aerial_robot_ros_compat REQUIRED)
find_package(angles REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

set(AERIAL_ROBOT_CONTROL_DEPS
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  aerial_robot_ros_compat
  angles
  geographic_msgs
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  sensor_msgs
  spinal
  std_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  visualization_msgs
)

# Eigen requires optimization to get good performance
# http://eigen.tuxfamily.org/index.php?title=FAQ#Optimization
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  # aerial_robot_estimation exports targets but its own headers are not on those
  # targets' interface include dirs, so bring package include dirs explicitly.
  ${aerial_robot_estimation_INCLUDE_DIRS}
)

add_library(control_utils SHARED
  src/control/utils/care.cpp)

add_library(flight_control_pluginlib SHARED
  src/control/base/pose_linear_controller.cpp
  src/control/fully_actuated_controller.cpp
  src/control/under_actuated_controller.cpp
  src/control/under_actuated_lqi_controller.cpp
  src/control/under_actuated_tilted_lqi_controller.cpp)
ament_target_dependencies(flight_control_pluginlib ${AERIAL_ROBOT_CONTROL_DEPS})
target_link_libraries(flight_control_pluginlib control_utils)

add_library(flight_navigation SHARED
  src/flight_navigation.cpp
  src/util/joy_parser.cpp)
ament_target_dependencies(flight_navigation ${AERIAL_ROBOT_CONTROL_DEPS})

add_library(trajectory_generation SHARED
  src/trajectory/reference_base.cpp
  src/trajectory/trajectory_reference/polynomial.cpp
  src/trajectory/trajectory_reference/polynomial_trajectory.cpp
  src/trajectory/trajectory_reference/sampled_trajectory.cpp
  src/trajectory/base/parameter_base.cpp
  src/trajectory/math/math.cpp
  src/trajectory/types/command.cpp
  src/trajectory/types/quad_state.cpp
  src/trajectory/types/quadrotor.cpp
  src/trajectory/utils/logger.cpp
  src/trajectory/utils/timer.cpp)

pluginlib_export_plugin_description_file(aerial_robot_control plugins/flight_control_plugins.ros2.xml)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS control_utils flight_control_pluginlib flight_navigation trajectory_generation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY scripts plugins
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AERIAL_ROBOT_CONTROL_DEPS})

ament_package()
