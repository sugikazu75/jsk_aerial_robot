# ROS2 build of dragon.
#
# Covers the four pluginlib libraries - sensor plugin, robot models, flight
# controllers, navigator - and the config, description and MuJoCo models. The
# numerical jacobian test library and its gtest binary are left out, as hydrus's
# and aerial_robot_model's already are, and so is the desktop-file install
# target, which is a developer convenience that shells out to a script.
#
# NLopt comes from aerial_robot_3rdparty, which builds it through
# ExternalProject; that package declares `build_type: cmake` and builds under
# colcon unchanged.

add_compile_options(-std=c++17)
add_definitions(-DAERIAL_ROBOT_ROS_VERSION=2)

find_package(ament_cmake REQUIRED)

find_package(aerial_robot_control REQUIRED)
find_package(aerial_robot_estimation REQUIRED)
find_package(aerial_robot_model REQUIRED)
find_package(aerial_robot_msgs REQUIRED)
find_package(aerial_robot_ros_compat REQUIRED)
find_package(hydrus REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(spinal REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)

set(DRAGON_DEPS
  aerial_robot_control
  aerial_robot_estimation
  aerial_robot_model
  aerial_robot_msgs
  aerial_robot_ros_compat
  hydrus
  geometry_msgs
  nav_msgs
  pluginlib
  rclcpp
  sensor_msgs
  spinal
  std_msgs
  std_srvs
  tf2
  visualization_msgs
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DNDEBUG")

include_directories(
  include
  # aerial_robot_control and aerial_robot_estimation export their targets, but
  # their headers are not always propagated onto consumers' compile lines.
  ${aerial_robot_control_INCLUDE_DIRS}
  ${aerial_robot_estimation_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS}
)

add_library(dragon_sensor_pluginlib SHARED src/sensor/imu.cpp)
ament_target_dependencies(dragon_sensor_pluginlib ${DRAGON_DEPS})
target_link_libraries(dragon_sensor_pluginlib aerial_robot_estimation::sensor_pluginlib)

add_library(dragon_robot_model SHARED
  src/model/hydrus_like_robot_model.cpp
  src/model/full_vectoring_robot_model.cpp)
ament_target_dependencies(dragon_robot_model ${DRAGON_DEPS})
target_link_libraries(dragon_robot_model
  hydrus::hydrus_robot_model
  aerial_robot_model::aerial_robot_model_lib
  ${NLOPT_LIBRARIES})

add_library(dragon_navigation SHARED src/dragon_navigation.cpp)
ament_target_dependencies(dragon_navigation ${DRAGON_DEPS})
target_link_libraries(dragon_navigation aerial_robot_control::flight_navigation)

add_library(dragon_aerial_robot_controllib SHARED
  src/control/lqi_gimbal_control.cpp
  src/control/full_vectoring_control.cpp)
ament_target_dependencies(dragon_aerial_robot_controllib ${DRAGON_DEPS})
target_link_libraries(dragon_aerial_robot_controllib
  dragon_robot_model
  dragon_navigation
  dragon_sensor_pluginlib
  hydrus::hydrus_controller_pluginlib
  aerial_robot_control::flight_control_pluginlib)

# On the targets, not just the directory: ament_target_dependencies picks a
# dependency's include directories up from its exported targets.
foreach(_target dragon_sensor_pluginlib dragon_robot_model dragon_navigation
                dragon_aerial_robot_controllib)
  target_include_directories(${_target} PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
endforeach()

pluginlib_export_plugin_description_file(aerial_robot_model plugins/robot_model_plugins.ros2.xml)
pluginlib_export_plugin_description_file(aerial_robot_control plugins/flight_control_plugins.ros2.xml)
pluginlib_export_plugin_description_file(aerial_robot_control plugins/flight_navigation_plugin.ros2.xml)
pluginlib_export_plugin_description_file(aerial_robot_estimation plugins/sensor_plugins.ros2.xml)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME})

install(TARGETS dragon_sensor_pluginlib dragon_robot_model dragon_navigation
                dragon_aerial_robot_controllib
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

# The generated mujoco/<...>/robot.xml pulls in the shared world through a path
# relative to the *source* tree, and that relationship does not survive
# installation. Resolve it here, where the source layout is still real, and
# install a self-contained model directory per variant: robot.xml with a bare
# <include file="world.xml"/>, and the world beside it.
file(GLOB_RECURSE MUJOCO_ROBOT_SOURCES
  RELATIVE ${PROJECT_SOURCE_DIR}/mujoco
  ${PROJECT_SOURCE_DIR}/mujoco/*/*/robot.xml)
if(NOT MUJOCO_ROBOT_SOURCES)
  message(FATAL_ERROR
    "No mujoco/*/*/robot.xml found. They are generated by the ROS1 build "
    "(mujoco_model_convert); run catkin build once before building for ROS2.")
endif()

foreach(_rel ${MUJOCO_ROBOT_SOURCES})
  set(_src ${PROJECT_SOURCE_DIR}/mujoco/${_rel})
  get_filename_component(_dir ${_rel} DIRECTORY)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_src})

  file(READ ${_src} _xml)
  string(REGEX MATCH "<include file=\"([^\"]*)\"" _matched "${_xml}")
  if(NOT CMAKE_MATCH_1)
    message(FATAL_ERROR "No <include file=...> found in ${_src}")
  endif()
  get_filename_component(_world "${PROJECT_SOURCE_DIR}/mujoco/${_dir}/${CMAKE_MATCH_1}" ABSOLUTE)
  if(NOT EXISTS ${_world})
    message(FATAL_ERROR "MuJoCo world ${_world} does not exist")
  endif()

  string(REGEX REPLACE "<include file=\"[^\"]*\"" "<include file=\"world.xml\"" _xml "${_xml}")
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/mujoco/${_rel} "${_xml}")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/mujoco/${_rel} ${_world}
    DESTINATION share/${PROJECT_NAME}/mujoco/${_dir})
endforeach()

# The meshes the models reference sit beside them.
foreach(_rel ${MUJOCO_ROBOT_SOURCES})
  get_filename_component(_dir ${_rel} DIRECTORY)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/mujoco/${_dir}/
    DESTINATION share/${PROJECT_NAME}/mujoco/${_dir}
    FILES_MATCHING PATTERN "*.stl" PATTERN "*.STL" PATTERN "*.dae")
endforeach()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DRAGON_DEPS})

ament_package()
