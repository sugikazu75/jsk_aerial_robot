// -*- mode: c++ -*-
//
// Lets the firmware's SIMULATION build compile against ROS2.
//
// The flight control and state estimation sources under mcu_project are built
// three ways: for the MCU against rosserial, and - with -DSIMULATION - on the
// host, where they talk to plain roscpp. That host build is what the MuJoCo and
// Gazebo simulations run, so it has to work under ROS2 as well.
//
// Rather than editing those sources (attitude_control.cpp alone is 1200 lines,
// and they are shared with the flight controller firmware, which must not be
// disturbed), this header supplies the names their SIMULATION path expects:
// the `ros::` types, backed by the compat layer, and this package's own
// messages hoisted from `spinal::msg` up into `spinal`, which is where ROS1
// generated them.
//
// Injecting into `ros::` is safe *here and only here*: it happens only when
// building for ROS2, where roscpp does not exist and there is nothing to
// collide with. Under ROS1 this header is empty and the firmware sees the real
// roscpp. That is the opposite of the rule the rest of the stack follows - see
// docs/ros2_migration.md - and the reason for the difference is that everywhere
// else roscpp is genuinely present.

#pragma once

#include <aerial_robot_ros_compat/ros_compat.h>

#if AERIAL_ROBOT_ROS_VERSION == 2

#include <spinal/msg/barometer.hpp>
#include <spinal/msg/desire_coord.hpp>
#include <spinal/msg/esc_telemetry_array.hpp>
#include <spinal/msg/flight_config_cmd.hpp>
#include <spinal/msg/four_axis_command.hpp>
#include <spinal/msg/gyro.hpp>
#include <spinal/msg/imu.hpp>
#include <spinal/msg/motor_info.hpp>
#include <spinal/msg/p_matrix_pseudo_inverse_with_inertia.hpp>
#include <spinal/msg/pwm_info.hpp>
#include <spinal/msg/pwm_test.hpp>
#include <spinal/msg/pwms.hpp>
#include <spinal/msg/roll_pitch_yaw_term.hpp>
#include <spinal/msg/roll_pitch_yaw_terms.hpp>
#include <spinal/msg/torque_allocation_matrix_inv.hpp>
#include <spinal/msg/uav_info.hpp>
#include <spinal/srv/mag_declination.hpp>

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace ros
{

using NodeHandle = ::ros_compat::NodeHandle;
using Publisher = ::ros_compat::Publisher;
using Subscriber = ::ros_compat::Subscriber;
using ServiceServer = ::ros_compat::ServiceServer;
using Time = ::ros_compat::Time;
using Duration = ::ros_compat::Duration;

}  // namespace ros

namespace spinal
{

// ROS1 generated these directly into `spinal`; ROS2 puts them in `spinal::msg`
// and `spinal::srv`. The firmware spells them unqualified, so hoist them.
using msg::Barometer;
using msg::DesireCoord;
using msg::ESCTelemetryArray;
using msg::FlightConfigCmd;
using msg::FourAxisCommand;
using msg::Gyro;
using msg::Imu;
using msg::MotorInfo;
using msg::PMatrixPseudoInverseWithInertia;
using msg::PwmInfo;
using msg::PwmTest;
using msg::Pwms;
using msg::RollPitchYawTerm;
using msg::RollPitchYawTerms;
using msg::TorqueAllocationMatrixInv;
using msg::UavInfo;

using srv::MagDeclination;

}  // namespace spinal

namespace geometry_msgs
{
using msg::Vector3Stamped;
}

namespace std_msgs
{
using msg::Float32;
using msg::Float32MultiArray;
using msg::UInt8;
}

namespace sensor_msgs
{
using msg::JointState;
}

namespace std_srvs
{
using srv::SetBool;
}

namespace ros_compat
{
// The firmware calls nh->advertiseService(name, &C::cb, this) without naming the
// service, which roscpp could deduce and rclcpp cannot. Register the mapping.
template <>
struct ServiceOf<std_srvs::srv::SetBool::Request>
{
  using type = std_srvs::srv::SetBool;
};
template <>
struct ServiceOf<spinal::srv::MagDeclination::Request>
{
  using type = spinal::srv::MagDeclination;
};
}  // namespace ros_compat

// rosconsole macros the firmware uses; rclcpp has no ROS_ prefixed forms.
#define ROS_DEBUG(...) ROS_COMPAT_DEBUG(__VA_ARGS__)
#define ROS_INFO(...) ROS_COMPAT_INFO(__VA_ARGS__)
#define ROS_WARN(...) ROS_COMPAT_WARN(__VA_ARGS__)
#define ROS_ERROR(...) ROS_COMPAT_ERROR(__VA_ARGS__)
#define ROS_FATAL(...) ROS_COMPAT_FATAL(__VA_ARGS__)

#endif  // AERIAL_ROBOT_ROS_VERSION == 2
