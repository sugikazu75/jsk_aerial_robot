// -*- mode: c++ -*-
//
// ROS1 backing for the ros_compat layer.
//
// Everything here is a plain alias onto roscpp. No wrapper objects, no extra
// indirection: under ROS1 `ros_compat::NodeHandle` *is* `ros::NodeHandle`, so
// the ROS1 build keeps exactly the behaviour it had before the migration and
// the shim cannot introduce a regression on the robots that are flying today.
//
// The ROS2 counterpart in ../ros_two/impl.h has to do real work to reproduce
// this API; when the two drift, this file is the specification.

#pragma once

#include <ros/ros.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <string>
#include <utility>

namespace ros_compat
{

using NodeHandle = ::ros::NodeHandle;
using Publisher = ::ros::Publisher;
using Subscriber = ::ros::Subscriber;
using ServiceServer = ::ros::ServiceServer;
// ServiceClient is a class template below, not an alias: ROS2 has no combined
// service type, so the client has to be parameterised on it.
using Timer = ::ros::Timer;
using TimerEvent = ::ros::TimerEvent;
using Time = ::ros::Time;
using Duration = ::ros::Duration;
using Rate = ::ros::Rate;
using AsyncSpinner = ::ros::AsyncSpinner;
using CallbackQueue = ::ros::CallbackQueue;
using TransportHints = ::ros::TransportHints;

/**
 * Shared pointer flavour used across the stack's interfaces.
 *
 * ROS1 pluginlib hands back boost::shared_ptr and the plugin base classes take
 * boost::shared_ptr in their initialize() signatures; ROS2 uses std::shared_ptr
 * throughout. The two are not interchangeable, so call sites spell the pointer
 * this way and get the right one.
 */
template <class T>
using SharedPtr = boost::shared_ptr<T>;

template <class T, class... Args>
SharedPtr<T> makeShared(Args&&... args)
{
  return boost::make_shared<T>(std::forward<Args>(args)...);
}

/** ROS1 pluginlib spells this createInstance; ROS2 dropped it for createSharedInstance. */
template <class Loader>
auto createPluginInstance(Loader& loader, const std::string& name) -> decltype(loader.createInstance(name))
{
  return loader.createInstance(name);
}

/**
 * Base for classes handing out a SharedPtr to themselves.
 *
 * Has to match whichever smart pointer SharedPtr is: shared_from_this() on a
 * boost base returns a boost::shared_ptr, which will not convert to a
 * std::shared_ptr.
 */
template <class T>
using EnableSharedFromThis = boost::enable_shared_from_this<T>;

/**
 * Blocking service client.
 *
 * Spelled with separate request and response because ROS2 has no combined
 * service type to pass around; roscpp's own call(req, res) overload takes the
 * same shape, so this is a thin pass-through here.
 */
template <class S>
class ServiceClient
{
public:
  ServiceClient() = default;
  ServiceClient(::ros::NodeHandle nh, const std::string& name) : impl_(nh.serviceClient<S>(name)), name_(name)
  {
  }

  bool exists(double = 0.0)
  {
    return impl_.exists();
  }

  bool call(typename S::Request& req, typename S::Response& res, double = 5.0)
  {
    return impl_.call(req, res);
  }

private:
  ::ros::ServiceClient impl_;
  std::string name_;
};

/** boost::dynamic_pointer_cast under ROS1, std:: under ROS2. */
template <class T, class U>
SharedPtr<T> dynamicPointerCast(const SharedPtr<U>& p)
{
  return boost::dynamic_pointer_cast<T>(p);
}

/** Shared pointer to a const message, as the callbacks in this stack take it. */
template <class M>
using ConstPtr = typename M::ConstPtr;

/** Shared pointer to a mutable message. */
template <class M>
using Ptr = typename M::Ptr;

inline void init(int& argc, char** argv, const std::string& name)
{
  ::ros::init(argc, argv, name);
}

inline bool ok()
{
  return ::ros::ok();
}
inline void spin()
{
  ::ros::spin();
}
inline void spinOnce()
{
  ::ros::spinOnce();
}
inline void shutdown()
{
  ::ros::shutdown();
}

inline Time now()
{
  return ::ros::Time::now();
}

/**
 * No-op under ROS1.
 *
 * The ROS2 build needs the process to hand its node to the compat layer before
 * ros_compat::now() can work, because ROS2 has no global clock. Calling this
 * unconditionally keeps the call sites version-agnostic.
 */
inline void setGlobalNode(const NodeHandle&)
{
}

}  // namespace ros_compat

// Logging. Under ROS1 these forward to rosconsole unchanged, which keeps the
// existing ROSCONSOLE_FORMAT / rqt_console behaviour and the per-node logger
// names that the users' launch files rely on.
#define ROS_COMPAT_DEBUG(...) ROS_DEBUG(__VA_ARGS__)
#define ROS_COMPAT_INFO(...) ROS_INFO(__VA_ARGS__)
#define ROS_COMPAT_WARN(...) ROS_WARN(__VA_ARGS__)
#define ROS_COMPAT_ERROR(...) ROS_ERROR(__VA_ARGS__)
#define ROS_COMPAT_FATAL(...) ROS_FATAL(__VA_ARGS__)

#define ROS_COMPAT_DEBUG_STREAM(args) ROS_DEBUG_STREAM(args)
#define ROS_COMPAT_INFO_STREAM(args) ROS_INFO_STREAM(args)
#define ROS_COMPAT_WARN_STREAM(args) ROS_WARN_STREAM(args)
#define ROS_COMPAT_ERROR_STREAM(args) ROS_ERROR_STREAM(args)
#define ROS_COMPAT_FATAL_STREAM(args) ROS_FATAL_STREAM(args)

#define ROS_COMPAT_INFO_THROTTLE(p, ...) ROS_INFO_THROTTLE(p, __VA_ARGS__)
#define ROS_COMPAT_WARN_THROTTLE(p, ...) ROS_WARN_THROTTLE(p, __VA_ARGS__)
#define ROS_COMPAT_ERROR_THROTTLE(p, ...) ROS_ERROR_THROTTLE(p, __VA_ARGS__)

#define ROS_COMPAT_INFO_ONCE(...) ROS_INFO_ONCE(__VA_ARGS__)
#define ROS_COMPAT_WARN_ONCE(...) ROS_WARN_ONCE(__VA_ARGS__)
#define ROS_COMPAT_ERROR_ONCE(...) ROS_ERROR_ONCE(__VA_ARGS__)

#define ROS_COMPAT_INFO_STREAM_ONCE(args) ROS_INFO_STREAM_ONCE(args)
#define ROS_COMPAT_WARN_STREAM_ONCE(args) ROS_WARN_STREAM_ONCE(args)
#define ROS_COMPAT_ERROR_STREAM_ONCE(args) ROS_ERROR_STREAM_ONCE(args)

#define ROS_COMPAT_INFO_COND(cond, ...) ROS_INFO_COND(cond, __VA_ARGS__)
#define ROS_COMPAT_WARN_COND(cond, ...) ROS_WARN_COND(cond, __VA_ARGS__)
#define ROS_COMPAT_ERROR_COND(cond, ...) ROS_ERROR_COND(cond, __VA_ARGS__)
