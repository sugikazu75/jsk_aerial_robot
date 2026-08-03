// -*- mode: c++ -*-
//
// Message include paths and type names differ between the two ROS versions and
// nothing in the compat layer can hide that at the call site:
//
//   ROS1   #include <spinal/Imu.h>          spinal::Imu
//   ROS2   #include <spinal/msg/imu.hpp>    spinal::msg::Imu
//
// The include paths have to be written out under an #if. Building them from
// macro arguments does not work: the preprocessor separates tokens with
// whitespace when it forms a header-name, so <pkg/Camel.h> expands to
// <geometry_msgs/ Vector3Stamped.h> and the include fails. Spelling both forms
// explicitly is also easier to grep when hunting down a message dependency.
//
// The type names, on the other hand, differ only by the extra `msg` / `srv`
// namespace, so a per-package alias covers every call site at once. The
// convention is:
//
//     #if AERIAL_ROBOT_ROS_VERSION == 1
//     #  include <spinal/Imu.h>
//     #  include <spinal/PwmInfo.h>
//     #else
//     #  include <spinal/msg/imu.hpp>
//     #  include <spinal/msg/pwm_info.hpp>
//     #endif
//     AERIAL_ROBOT_MSG_NAMESPACE(spinal);
//
// after which call sites say `spinal_c::Imu` and compile either way.
//
// Aliasing rather than `using namespace` keeps ROS2's `spinal::msg` and ROS1's
// `spinal` from ever being visible under the same name at once, which is what
// makes this safe to mix with code that still includes roscpp directly.

#pragma once

#include <aerial_robot_ros_compat/ros_compat.h>

/**
 * Declare the `<pkg>_c` alias for a message package's type namespace, and
 * `<pkg>_s` for its service namespace.
 *
 *   AERIAL_ROBOT_MSG_NAMESPACE(spinal);   ->  namespace spinal_c = spinal;       (ROS1)
 *                                             namespace spinal_c = spinal::msg;  (ROS2)
 */
#if AERIAL_ROBOT_ROS_VERSION == 1
#define AERIAL_ROBOT_MSG_NAMESPACE(pkg) namespace pkg##_c = pkg
#define AERIAL_ROBOT_SRV_NAMESPACE(pkg) namespace pkg##_s = pkg
#else
#define AERIAL_ROBOT_MSG_NAMESPACE(pkg) namespace pkg##_c = pkg::msg
#define AERIAL_ROBOT_SRV_NAMESPACE(pkg) namespace pkg##_s = pkg::srv
#endif
