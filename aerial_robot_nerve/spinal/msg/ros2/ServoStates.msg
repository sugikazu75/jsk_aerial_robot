# ROS2 variant of ../ServoStates.msg
# ROS1 uses the bare 'time' primitive, which rosidl has no IDL mapping for.
# Keep this file in sync with ../ServoStates.msg; only the stamp field differs.
builtin_interfaces/Time stamp
spinal/ServoState[] servos
