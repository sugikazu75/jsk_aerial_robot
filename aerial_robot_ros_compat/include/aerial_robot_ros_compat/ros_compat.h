// -*- mode: c++ -*-
//
// Single entry point for the ROS1/ROS2 compatibility layer.
//
// Include this instead of <ros/ros.h> and spell the ROS types
// `ros_compat::NodeHandle`, `ros_compat::Publisher`, ... The layer deliberately
// does NOT inject anything into the `ros::` namespace: under ROS1 roscpp is
// still included and owns that namespace, and shadowing it would make it
// ambiguous which implementation a given call site resolves to.
//
// Under ROS1 every name here is a plain alias onto roscpp (see ros_one/impl.h),
// so the ROS1 build is unchanged. Under ROS2 they are rclcpp-backed
// reimplementations (see ros_two/impl.h).

#pragma once

// Which ROS this translation unit is being built against.
//
// Normally the build system says so: every hybrid package's cmake/Ros1.cmake
// and cmake/Ros2.cmake pass -DAERIAL_ROBOT_ROS_VERSION. Note that ROS_VERSION
// is only an *environment* variable, so it is not visible to the preprocessor
// and cannot be tested here.
//
// The __has_include fallback covers targets that forget to pass the define. It
// is reliable even on a machine with both distributions installed, because they
// live under separate prefixes and only the sourced one lands on the compiler's
// include path.
#if !defined(AERIAL_ROBOT_ROS_VERSION)
#if defined(__has_include)
#if __has_include(<rclcpp/rclcpp.hpp>)
#define AERIAL_ROBOT_ROS_VERSION 2
#else
#define AERIAL_ROBOT_ROS_VERSION 1
#endif
#else
#define AERIAL_ROBOT_ROS_VERSION 1
#endif
#endif

#if AERIAL_ROBOT_ROS_VERSION == 1
#include <aerial_robot_ros_compat/ros_one/impl.h>
#else
#include <aerial_robot_ros_compat/ros_two/impl.h>
#endif

namespace ros_compat
{

/**
 * Read a parameter into `param`, falling back to `default_value`.
 *
 * The stack had three copies of this helper (flight_navigation.h,
 * control/base/base.h, sensor/base_plugin.h); they are unified here so the
 * ROS1/ROS2 difference is expressed once.
 */
template <class T>
bool getParam(const NodeHandle& nh, const std::string& name, T& param, const T& default_value)
{
  return const_cast<NodeHandle&>(nh).param(name, param, default_value);
}

// ---------------------------------------------------------------------------
// Free functions for the places where roscpp and rclcpp cannot share a spelling
// even after aliasing. Call sites use these instead of the member functions.
// ---------------------------------------------------------------------------

/**
 * Advertise a service.
 *
 * Spelled as a free function taking the service type explicitly because the two
 * APIs need different information: roscpp deduces request/response from the
 * callback, while rclcpp needs the service type itself, and a ROS2 request
 * struct carries no way back to the service that owns it.
 */
template <class S, class T>
ServiceServer advertiseService(NodeHandle& nh, const std::string& name,
                               bool (T::*fp)(typename S::Request&, typename S::Response&), T* obj)
{
#if AERIAL_ROBOT_ROS_VERSION == 1
  return nh.advertiseService(name, fp, obj);
#else
  return nh.template advertiseService<S>(name, fp, obj);
#endif
}

/** Duration from seconds. rclcpp::Duration has no double constructor. */
inline Duration duration(double seconds)
{
#if AERIAL_ROBOT_ROS_VERSION == 1
  return Duration(seconds);
#else
  return Duration::from_seconds(seconds);
#endif
}

/** Time from seconds. */
inline Time time(double seconds)
{
#if AERIAL_ROBOT_ROS_VERSION == 1
  return Time(seconds);
#else
  return Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
#endif
}

/** Seconds held by a Time, spelled the same on both sides. */
inline double toSec(const Time& t)
{
#if AERIAL_ROBOT_ROS_VERSION == 1
  return t.toSec();
#else
  return t.seconds();
#endif
}

/** Seconds held by a Duration. */
inline double toSec(const Duration& d)
{
#if AERIAL_ROBOT_ROS_VERSION == 1
  return d.toSec();
#else
  return d.seconds();
#endif
}

/**
 * Bring up the process and return its root handle.
 *
 * Under ROS1 this is ros::init plus a default NodeHandle. Under ROS2 it also
 * creates the node object, which ROS1 has no equivalent of, and registers it as
 * the process-global node so that now() and the logging macros have a clock and
 * a logger.
 */
inline NodeHandle initNode(int& argc, char** argv, const std::string& name)
{
#if AERIAL_ROBOT_ROS_VERSION == 1
  ::ros::init(argc, argv, name);
  return NodeHandle();
#else
  rclcpp::init(argc, argv);
  NodeHandle nh(createNode(name));
  setGlobalNode(nh);
  return nh;
#endif
}

/**
 * Join a tf_prefix onto a frame name, as tf::resolve did.
 *
 * ROS2 removed tf_prefix, but the launch files still namespace each robot and
 * pass a tf_prefix parameter, so the concept is still needed. This is pure
 * string handling and identical on both sides.
 *
 * Behaviour verified against ROS1's tf::resolve:
 *   ("",           "cog")  -> "cog"
 *   ("quadrotor1", "cog")  -> "quadrotor1/cog"
 *   ("quadrotor1", "/cog") -> "cog"     absolute frame ignores the prefix
 *   ("/quadrotor1","cog")  -> "quadrotor1/cog"   leading slash stripped
 */
inline std::string resolveFrame(const std::string& prefix, const std::string& frame_name)
{
  auto strip_leading_slash = [](const std::string& s) { return (!s.empty() && s.front() == '/') ? s.substr(1) : s; };

  if (!frame_name.empty() && frame_name.front() == '/')
    return strip_leading_slash(frame_name);
  if (prefix.empty())
    return frame_name;
  return strip_leading_slash(prefix) + "/" + frame_name;
}

/**
 * Construct a tf2_ros broadcaster.
 *
 * tf2_ros exists under both versions, but ROS1's broadcasters default-construct
 * while ROS2's need the node. Kept as a template so this header does not have to
 * depend on tf2_ros; it is only instantiated where a broadcaster is actually
 * declared. Returns a shared_ptr because the ROS2 types are not default
 * constructible and so cannot be plain members initialised later.
 */
template <class Broadcaster>
std::shared_ptr<Broadcaster> makeBroadcaster(const NodeHandle& nh)
{
#if AERIAL_ROBOT_ROS_VERSION == 1
  (void)nh;
  return std::make_shared<Broadcaster>();
#else
  return std::make_shared<Broadcaster>(nh.node());
#endif
}

/** The roscpp "~" handle. */
inline NodeHandle privateNodeHandle(const NodeHandle& nh)
{
#if AERIAL_ROBOT_ROS_VERSION == 1
  (void)nh;
  return NodeHandle("~");
#else
  return nh.privateHandle();
#endif
}

}  // namespace ros_compat
