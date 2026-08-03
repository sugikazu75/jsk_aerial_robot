# ROS2 build of aerial_robot_model.
#
# Deliberately narrower than the ROS1 build (see Ros1.cmake). It covers what
# the mini_quadrotor MuJoCo bringup launches - the model libraries, the
# pluginlib plugins and rotor_tf_publisher - and leaves servo_bridge,
# interactive_marker_tf_broadcaster and the numerical_jacobians test out.
# Those still speak roscpp directly and none of them start for that robot, so
# porting them is not on the path to the first milestone.

add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=2)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(aerial_robot_ros_compat REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kalman_filter REQUIRED)
find_package(kdl_parser REQUIRED)
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
find_package(urdf REQUIRED)

find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 REQUIRED)

# tinyxml2 ships no CMake config package on this platform, so go through
# pkg-config. Used instead of ROS1's TinyXML1, which ROS2 does not have.
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED tinyxml2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

# The interface target has to take the project name for ament to index it,
# which is why the model library is aerial_robot_model_lib here and plain
# aerial_robot_model under catkin.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddExtraModule.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
# The libraries below use this package's own service, so they link the
# generated typesupport rather than a separate messages package.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(AERIAL_ROBOT_MODEL_DEPS
  aerial_robot_ros_compat
  geometry_msgs
  kdl_parser
  pluginlib
  rclcpp
  sensor_msgs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_kdl
  tf2_ros
  urdf
)

include_directories(
  include
  ${TINYXML2_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(aerial_robot_model_lib SHARED
  src/model/base_model/robot_model.cpp
  src/model/transformable_model/robot_model.cpp
  src/model/transformable_model/jacobians.cpp
  src/model/transformable_model/kinematics.cpp
  src/model/transformable_model/stability.cpp
  src/model/transformable_model/statics.cpp)
ament_target_dependencies(aerial_robot_model_lib ${AERIAL_ROBOT_MODEL_DEPS})
target_link_libraries(aerial_robot_model_lib
  ${orocos_kdl_LIBRARIES} ${TINYXML2_LIBRARIES} "${cpp_typesupport_target}")

add_library(aerial_robot_model_ros SHARED src/model/base_model/robot_model_ros.cpp)
ament_target_dependencies(aerial_robot_model_ros ${AERIAL_ROBOT_MODEL_DEPS} spinal)
target_link_libraries(aerial_robot_model_ros
  aerial_robot_model_lib ${orocos_kdl_LIBRARIES} "${cpp_typesupport_target}")

add_library(robot_model_pluginlib SHARED
  src/model/plugin/multirotor_robot_model.cpp
  src/model/plugin/underactuated_tilted_robot_model.cpp)
ament_target_dependencies(robot_model_pluginlib ${AERIAL_ROBOT_MODEL_DEPS})
target_link_libraries(robot_model_pluginlib aerial_robot_model_lib)

add_executable(rotor_tf_publisher src/utils/rotor_tf_publisher.cpp)
ament_target_dependencies(rotor_tf_publisher ${AERIAL_ROBOT_MODEL_DEPS})
target_link_libraries(rotor_tf_publisher ${orocos_kdl_LIBRARIES})

pluginlib_export_plugin_description_file(aerial_robot_model plugins/robot_model_plugins.xml)

# Nested one level deeper than it looks: a package that also generates
# interfaces gets include/<pkg> on its consumers' include path, because that is
# where rosidl puts the generated headers. The hand-written headers have to sit
# under the same root for <aerial_robot_model/model/...> to resolve.
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME})

install(TARGETS aerial_robot_model_lib aerial_robot_model_ros robot_model_pluginlib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS rotor_tf_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch script plugins
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${AERIAL_ROBOT_MODEL_DEPS} rosidl_default_runtime)

ament_package()
