# ROS2 build of aerial_robot_estimation.
#
# Narrower than the ROS1 build (see Ros1.cmake): the state estimator plus the
# imu, mocap and vo sensor plugins, which is what mini_quadrotor's
# StateEstimation.yaml loads. Left out for now, none of them on the path to the
# first milestone:
#
#   src/sensor/gps.cpp             needs geodesy / geographic_msgs
#   src/sensor/plane_detection.cpp needs jsk_recognition_msgs
#   src/sensor/altitude.cpp        pulls in the barometer bias filter
#   src/kf/*                       xy_roll_pitch_bias is dynamic_reconfigure
#   src/vision/optical_flow.cpp    is a nodelet, which ROS2 replaced with
#                                  components
#
# dynamic_reconfigure and nodelet are the only two ROS1-only frameworks this
# package uses, and both happen to sit entirely inside that excluded set.

add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=2)

find_package(ament_cmake REQUIRED)

find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(aerial_robot_ros_compat REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kalman_filter REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(Eigen3 REQUIRED)

set(AERIAL_ROBOT_ESTIMATION_DEPS
  aerial_robot_model
  aerial_robot_msgs
  aerial_robot_ros_compat
  geographic_msgs
  geometry_msgs
  kalman_filter
  nav_msgs
  pluginlib
  rclcpp
  sensor_msgs
  spinal
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
)

include_directories(include ${EIGEN3_INCLUDE_DIRS})

add_library(aerial_robot_estimation SHARED src/state_estimation.cpp)
ament_target_dependencies(aerial_robot_estimation ${AERIAL_ROBOT_ESTIMATION_DEPS})

add_library(sensor_pluginlib SHARED
  src/sensor/imu.cpp
  src/sensor/mocap.cpp
  src/sensor/vo.cpp)
ament_target_dependencies(sensor_pluginlib ${AERIAL_ROBOT_ESTIMATION_DEPS})
target_link_libraries(sensor_pluginlib aerial_robot_estimation)

pluginlib_export_plugin_description_file(aerial_robot_estimation plugins/sensor_plugins.ros2.xml)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS aerial_robot_estimation sensor_pluginlib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY launch plugins config
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AERIAL_ROBOT_ESTIMATION_DEPS})

ament_package()
