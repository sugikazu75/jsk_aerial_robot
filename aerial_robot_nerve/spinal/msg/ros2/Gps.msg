# ROS2 variant of ../Gps.msg
# ROS1 uses the bare 'time' primitive, which rosidl has no IDL mapping for.
# Keep this file in sync with ../Gps.msg; only the stamp field differs.
builtin_interfaces/Time stamp
float64[2] location
float32[2] velocity
uint8 sat_num
