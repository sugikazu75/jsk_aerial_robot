# ROS2 variant of ../Barometer.msg
# ROS1 uses the bare 'time' primitive, which rosidl has no IDL mapping for.
# Keep this file in sync with ../Barometer.msg; only the stamp field differs.
builtin_interfaces/Time stamp

float32 altitude
float32 pressure
float32 temperature
