find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  )

add_message_files(FILES
  DynamicReconfigureLevels.msg
  FourAxisGain.msg
  AerialRobotStatus.msg
  FlightNav.msg
  PoseControlPid.msg
  Pid.msg
  State.msg
  States.msg
  Acc.msg
  WrenchAllocationMatrix.msg
  ApplyWrench.msg
  ForceList.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS std_msgs geometry_msgs message_runtime
)
