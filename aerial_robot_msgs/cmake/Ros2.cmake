find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamicReconfigureLevels.msg"
  "msg/FourAxisGain.msg"
  "msg/AerialRobotStatus.msg"
  "msg/FlightNav.msg"
  "msg/PoseControlPid.msg"
  "msg/Pid.msg"
  "msg/State.msg"
  "msg/States.msg"
  "msg/Acc.msg"
  "msg/WrenchAllocationMatrix.msg"
  "msg/ApplyWrench.msg"
  "msg/ForceList.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
