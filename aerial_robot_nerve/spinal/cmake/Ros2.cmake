# ROS2 build of the spinal package.
#
# spinal itself stays a ROS1 package: the host-side rosserial bridge, the rqt
# GUI plugin and the mcu_project firmware tooling are all rosserial-bound and
# are only built under catkin (see Ros1.cmake).  What ROS2 needs from spinal is
# the interface set, so that ROS2 nodes can reach the ROS1 flight controller
# through ros1_bridge.
#
# The package deliberately keeps the name "spinal" on both sides.  That makes
# ros1_bridge pair spinal/Imu with spinal/msg/Imu automatically, with no
# mapping_rules.yaml, and it keeps the "spinal/MotorInfo[] motor_info" style
# self-references inside the .msg files valid without any edit.
#
# Two definitions cannot be shared verbatim with ROS1:
#
#   * Seven messages declare their timestamp with the bare ROS1 "time"
#     primitive.  rosidl parses "time" but has no IDL mapping for it, so
#     generation dies with KeyError: 'time'.  Their ROS2 variants live in
#     msg/ros2/ and differ from the ROS1 files only in that one field.
#     They cannot be handed to rosidl_generate_interfaces() from that path
#     directly, because rosidl derives the interface namespace from the
#     immediate parent directory - "msg/ros2/Imu.msg" would register as
#     spinal/ros2/Imu.  They are staged into the build tree under a msg/
#     directory instead and passed using rosidl's "<base path>:<relative
#     path>" tuple form.
#
#   * SimpleImu.msg uses camelCase field names ("accData"), which rosidl
#     rejects outright.  It is firmware-only: the generated MCU headers under
#     mcu_project/boards/*/ros_lib are its sole consumers and nothing on the
#     host side publishes or subscribes to it, so it is left out of the ROS2
#     build.  If it ever has to cross the bridge it needs a snake_case ROS2
#     variant plus an explicit ros1_bridge mapping rule.

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Messages whose ROS2 variant lives in msg/ros2/, staged into the build tree so
# that rosidl sees them under a directory actually called "msg".
set(_spinal_ros2_only_msgs
  Imu
  Gyro
  Barometer
  Gps
  GpsFull
  ServoStates
  ESCTelemetryArray
)

set(_spinal_staging_dir "${CMAKE_CURRENT_BINARY_DIR}/ros2_interfaces")
set(_spinal_interfaces "")
foreach(_msg ${_spinal_ros2_only_msgs})
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/ros2/${_msg}.msg"
    "${_spinal_staging_dir}/msg/${_msg}.msg"
    COPYONLY)
  list(APPEND _spinal_interfaces "${_spinal_staging_dir}:msg/${_msg}.msg")
endforeach()

# Shared with ROS1 verbatim.
list(APPEND _spinal_interfaces
  "msg/PMatrixPseudoInverseUnit.msg"
  "msg/PMatrixPseudoInverseWithInertia.msg"
  "msg/ServoInfo.msg"
  "msg/BoardInfo.msg"
  "msg/ServoState.msg"
  "msg/ServoControlCmd.msg"
  "msg/ServoTorqueCmd.msg"
  "msg/ServoTorqueStates.msg"
  "msg/FourAxisCommand.msg"
  "msg/RollPitchYawTerm.msg"
  "msg/RollPitchYawTerms.msg"
  "msg/MotorInfo.msg"
  "msg/PwmInfo.msg"
  "msg/Pwms.msg"
  "msg/PwmTest.msg"
  "msg/UavInfo.msg"
  "msg/DesireCoord.msg"
  "msg/FlightConfigCmd.msg"
  "msg/Vector3Int16.msg"
  "msg/TorqueAllocationMatrixInv.msg"
  "msg/ESCTelemetry.msg"
  "msg/JointProfile.msg"
  "msg/JointProfiles.msg"

  "srv/GetBoardInfo.srv"
  "srv/SetBoardConfig.srv"
  "srv/SetAttitudeGains.srv"
  "srv/ImuCalib.srv"
  "srv/MagDeclination.srv"
  "srv/SetDirectServoConfig.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${_spinal_interfaces}
  DEPENDENCIES builtin_interfaces std_msgs
)

# ---------------------------------------------------------------------------
# The firmware's simulation build.
#
# attitude_control.cpp and the math library are compiled with -DSIMULATION for
# the host, where they drive the simulated flight controller. The MuJoCo
# hardware component in aerial_robot_simulation runs exactly this code, so it
# has to exist under ROS2 too. The sources are shared with the MCU firmware and
# are not edited: spinal_ros2_shim.h supplies the `ros::` names their SIMULATION
# path uses, backed by the compat layer.
# ---------------------------------------------------------------------------

find_package(aerial_robot_ros_compat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(SPINAL_DIRS mcu_project/lib/Jsk_Lib)

add_library(spinal_math SHARED
  ${SPINAL_DIRS}/math/AP_Math.cpp
  ${SPINAL_DIRS}/math/matrix3.cpp
  ${SPINAL_DIRS}/math/vector2.cpp
  ${SPINAL_DIRS}/math/vector3.cpp
  ${SPINAL_DIRS}/math/quaternion.cpp
  ${SPINAL_DIRS}/math/location.cpp)
target_include_directories(spinal_math PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${SPINAL_DIRS}>"
  "$<INSTALL_INTERFACE:include>")
target_compile_definitions(spinal_math PUBLIC SIMULATION)
ament_target_dependencies(spinal_math aerial_robot_ros_compat rclcpp)


add_library(spinal_flight_controller SHARED
  ${SPINAL_DIRS}/flight_control/attitude/attitude_control.cpp)
target_include_directories(spinal_flight_controller PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${SPINAL_DIRS}>"
  "$<INSTALL_INTERFACE:include>")
target_compile_definitions(spinal_flight_controller PUBLIC SIMULATION)
ament_target_dependencies(spinal_flight_controller
  aerial_robot_ros_compat rclcpp std_msgs std_srvs sensor_msgs geometry_msgs)

# The controller publishes and subscribes this package's own interfaces, so it
# links the generated typesupport directly.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(spinal_flight_controller spinal_math "${cpp_typesupport_target}")

install(DIRECTORY ${SPINAL_DIRS}/ DESTINATION include)

install(TARGETS spinal_math spinal_flight_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(aerial_robot_ros_compat rclcpp sensor_msgs geometry_msgs std_msgs std_srvs rosidl_default_runtime)
ament_package()
