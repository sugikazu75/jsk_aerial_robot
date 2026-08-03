# ROS2 variant of ../Gyro.msg
# ROS1 uses the bare 'time' primitive, which rosidl has no IDL mapping for.
# Keep this file in sync with ../Gyro.msg; only the stamp field differs.
builtin_interfaces/Time stamp
int16[6] x
int16[6] y
int16[6] z
