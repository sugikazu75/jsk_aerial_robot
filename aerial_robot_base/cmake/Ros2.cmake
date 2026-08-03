# ROS2 build of aerial_robot_base.
#
# Scope intentionally covers the C++ base node used to wire model, estimation,
# navigation and control together. The rospy/SMACH helper scripts remain in the
# ROS1 build for now.

add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=2)

find_package(ament_cmake REQUIRED)

find_package(aerial_robot_control REQUIRED)
find_package(aerial_robot_estimation REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_ros_compat REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(AERIAL_ROBOT_BASE_DEPS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_ros_compat
  pluginlib
  rclcpp
)

include_directories(include)

add_library(aerial_robot_base SHARED src/aerial_robot_base.cpp)
ament_target_dependencies(aerial_robot_base ${AERIAL_ROBOT_BASE_DEPS})

add_executable(aerial_robot_base_node src/aerial_robot_base_node.cpp)
ament_target_dependencies(aerial_robot_base_node ${AERIAL_ROBOT_BASE_DEPS})
target_link_libraries(aerial_robot_base_node aerial_robot_base)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS aerial_robot_base
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS aerial_robot_base_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AERIAL_ROBOT_BASE_DEPS})

ament_package()
