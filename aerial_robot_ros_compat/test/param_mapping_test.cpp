// Does a child NodeHandle actually read the nested config the launch files use?
//
// This is the single assumption the whole compat layer rests on. The stack is
// full of `NodeHandle child(nh_, "estimation"); child.param("mode", ...)`, and
// under ROS2 that has to come out as the parameter "estimation.mode". If the
// joining is wrong the code still compiles and still runs - every parameter
// just silently falls back to its default, which on a flying robot means
// default gains rather than a crash.
//
// Runs under both versions: ROS1 reads the same nesting from the parameter
// server, ROS2 from a --params-file.
#include <iostream>
#include <aerial_robot_ros_compat/ros_compat.h>
#include <cstdio>
#include <vector>
#include <string>

int fails = 0;
template <class T>
void check(const char* what, const T& got, const T& want)
{
  bool ok = (got == want);
  if (!ok)
    ++fails;
  std::cout << (ok ? "  ok   " : "  FAIL ") << what << "  got=" << got << " want=" << want << "\n";
}

int main(int argc, char** argv)
{
  auto nh = ros_compat::initNode(argc, argv, "param_probe");

  int uav_model = -1;
  ros_compat::getParam<int>(nh, "uav_model", uav_model, -1);
  check("uav_model (root)", uav_model, 7);

  ros_compat::NodeHandle est(nh, "estimation");
  int mode = -1;
  ros_compat::getParam<int>(est, "mode", mode, -1);
  check("estimation/mode (child)", mode, 2);

  ros_compat::NodeHandle motor(nh, "motor_info");
  double max_pwm = -1;
  ros_compat::getParam<double>(motor, "max_pwm", max_pwm, -1.0);
  check("motor_info/max_pwm (child)", max_pwm, 0.85);

  ros_compat::NodeHandle ref(motor, "ref1");
  double voltage = -1;
  ros_compat::getParam<double>(ref, "voltage", voltage, -1.0);
  check("motor_info/ref1/voltage (grandchild)", voltage, 11.1);

  std::vector<std::string> sensors;
  est.getParam("sensor_list", sensors);
  check("estimation/sensor_list size", (int)sensors.size(), 2);

  // roscpp's parameter server was untyped and widened a whole number to a
  // double on request; rclcpp throws instead. The config files are full of
  // gains written without a decimal point, so the compat layer has to widen.
  double max_thrust = -1;
  ros_compat::getParam<double>(ref, "max_thrust", max_thrust, -1.0);
  check("int-valued yaml read as double (grandchild)", max_thrust, 8.0);

  double whole_number_gain = -1;
  ros_compat::getParam<double>(nh, "whole_number_gain", whole_number_gain, -1.0);
  check("int-valued yaml read as double (root)", whole_number_gain, 3.0);

  // The reverse widening is allowed only when it loses nothing.
  int fractional_index = -1;
  check("double with a fraction refused as int",
        nh.getParam("fractional_index", fractional_index), false);

  check("hasParam on a present nested key", est.hasParam("mode"), true);
  check("hasParam on an absent key", est.hasParam("not_there"), false);

  std::printf(fails ? "\nPARAM MAPPING: %d FAILURES\n" : "\nPARAM MAPPING: all checks passed\n", fails);
  ros_compat::shutdown();
  return fails ? 1 : 0;
}
