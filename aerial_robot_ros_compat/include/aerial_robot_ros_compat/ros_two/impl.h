// -*- mode: c++ -*-
//
// ROS2 backing for the ros_compat layer.
//
// This reproduces the slice of the roscpp API that the aerial robot stack
// actually uses (see ../ros_one/impl.h for the specification) on top of rclcpp.
//
// The one concept with no ROS2 equivalent is ros::NodeHandle used as a
// *namespace handle*: the stack is full of
//
//     ros::NodeHandle control_nh(nh_, "controller");
//     getParam<double>(motor_nh, "max_pwm", max_pwm_, 0.0);
//
// ROS2 has a single flat, per-node parameter space and no child handles. So
// NodeHandle here is a lightweight value type holding a node pointer plus a
// namespace, and it joins that namespace onto every name it is asked about:
//
//     parameters -> dot-joined     "controller.max_pwm"
//     topics     -> slash-joined   "controller/max_pwm"
//
// Dot-joined parameters are what ROS2 YAML produces for nested mappings, so the
// existing nested config files keep their shape.
//
// IMPORTANT: for hasParam()/getParam() to answer correctly, the underlying node
// must be created with
//
//     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
//
// Without it every parameter supplied by YAML looks undeclared, has_parameter()
// returns false, and the many `if (nh.hasParam(x))` branches in this stack
// silently take their default path. createNode() below sets that option.

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace ros_compat
{

/**
 * rclcpp::Time and Duration with roscpp's accessors added.
 *
 * The stack calls .toSec() in about 140 places and .fromSec() in a further
 * seventeen. Rewriting each of those to a free function would be a large,
 * mechanical, and entirely reviewable-by-eye-only diff through the control and
 * estimation loops; deriving instead keeps the call sites readable and confines
 * the difference to here. They slice safely into the rclcpp types when handed
 * to rclcpp APIs, and the converting constructors pick results back up.
 */
class Duration : public rclcpp::Duration
{
public:
  using rclcpp::Duration::Duration;
  Duration(const rclcpp::Duration& d) : rclcpp::Duration(d)
  {
  }
  Duration() : rclcpp::Duration(0, 0)
  {
  }

  double toSec() const
  {
    return seconds();
  }
  void fromSec(double s)
  {
    *this = Duration(rclcpp::Duration::from_seconds(s));
  }

  static Duration fromSeconds(double s)
  {
    return Duration(rclcpp::Duration::from_seconds(s));
  }
};

class Time : public rclcpp::Time
{
public:
  using rclcpp::Time::Time;
  Time(const rclcpp::Time& t) : rclcpp::Time(t)
  {
  }

  double toSec() const
  {
    return seconds();
  }
  void fromSec(double s)
  {
    *this = Time(static_cast<int64_t>(s * 1e9), get_clock_type());
  }
  bool isZero() const
  {
    return nanoseconds() == 0;
  }

  Duration operator-(const Time& rhs) const
  {
    return Duration(static_cast<const rclcpp::Time&>(*this) - static_cast<const rclcpp::Time&>(rhs));
  }
  Time operator+(const Duration& rhs) const
  {
    return Time(static_cast<const rclcpp::Time&>(*this) + static_cast<const rclcpp::Duration&>(rhs));
  }
  Time operator-(const Duration& rhs) const
  {
    return Time(static_cast<const rclcpp::Time&>(*this) - static_cast<const rclcpp::Duration&>(rhs));
  }
};

using Rate = rclcpp::Rate;

/**
 * Shared pointer flavour used across the stack's interfaces.
 *
 * ROS1 pluginlib hands back boost::shared_ptr and the plugin base classes take
 * boost::shared_ptr in their initialize() signatures; ROS2 uses std::shared_ptr
 * throughout. The two are not interchangeable, so call sites spell the pointer
 * this way and get the right one.
 */
template <class T>
using SharedPtr = std::shared_ptr<T>;

template <class T, class... Args>
SharedPtr<T> makeShared(Args&&... args)
{
  return std::make_shared<T>(std::forward<Args>(args)...);
}

/** ROS1 pluginlib spells this createInstance; ROS2 dropped it for createSharedInstance. */
template <class Loader>
auto createPluginInstance(Loader& loader, const std::string& name) -> decltype(loader.createSharedInstance(name))
{
  return loader.createSharedInstance(name);
}

/**
 * Base for classes handing out a SharedPtr to themselves.
 *
 * Has to match whichever smart pointer SharedPtr is: shared_from_this() on a
 * boost base returns a boost::shared_ptr, which will not convert to a
 * std::shared_ptr.
 */
template <class T>
using EnableSharedFromThis = std::enable_shared_from_this<T>;

/** boost::dynamic_pointer_cast under ROS1, std:: under ROS2. */
template <class T, class U>
SharedPtr<T> dynamicPointerCast(const SharedPtr<U>& p)
{
  return std::dynamic_pointer_cast<T>(p);
}

template <class M>
using ConstPtr = typename M::ConstSharedPtr;

template <class M>
using Ptr = typename M::SharedPtr;

/**
 * Accepted for source compatibility with roscpp.
 *
 * ROS2 expresses the same intent through QoS rather than transport selection,
 * and these are no-ops. Note that means a subscriber asking for UDP - mocap
 * does - silently gets the default reliable transport instead. Matching the
 * old behaviour would mean a best-effort QoS profile, which is a decision for
 * whoever tunes the ROS2 mocap path against real hardware.
 */
struct TransportHints
{
  TransportHints& tcpNoDelay(bool = true)
  {
    return *this;
  }
  TransportHints& udp()
  {
    return *this;
  }
  TransportHints& tcp()
  {
    return *this;
  }
};

namespace detail
{

/**
 * Process-wide node used by the free functions that roscpp exposes globally
 * (now(), ok(), spin()). ROS2 has no global clock, so a process must publish its
 * node here once via setGlobalNode() before ros_compat::now() is meaningful.
 */
inline std::shared_ptr<rclcpp::Node>& globalNode()
{
  static std::shared_ptr<rclcpp::Node> node;
  return node;
}

/** Join a namespace and a relative name with `sep`, tolerating empty parts. */
inline std::string join(const std::string& ns, const std::string& name, char sep)
{
  if (ns.empty())
    return name;
  if (name.empty())
    return ns;
  if (name.front() == '/')
    return name;  // already absolute: roscpp semantics
  return ns + sep + name;
}

/** roscpp namespaces are slash-separated; ROS2 parameter names are dotted. */
inline std::string toParamName(const std::string& ns, const std::string& name)
{
  std::string joined = join(ns, name, '.');
  for (char& c : joined)
  {
    if (c == '/')
      c = '.';
  }
  while (!joined.empty() && joined.front() == '.')
    joined.erase(joined.begin());
  return joined;
}

}  // namespace detail

class Publisher
{
public:
  Publisher() = default;

  template <class PubT>
  explicit Publisher(std::shared_ptr<PubT> pub) : impl_(std::make_shared<Model<PubT>>(std::move(pub)))
  {
  }

  template <class M>
  void publish(const M& msg) const
  {
    if (!impl_)
      return;
    auto typed = std::dynamic_pointer_cast<Model<rclcpp::Publisher<M>>>(impl_);
    if (!typed)
      return;
    typed->pub_->publish(msg);
  }

  size_t getNumSubscribers() const
  {
    return impl_ ? impl_->subscriptionCount() : 0;
  }
  std::string getTopic() const
  {
    return impl_ ? impl_->topic() : std::string();
  }
  explicit operator bool() const
  {
    return static_cast<bool>(impl_);
  }

  void shutdown()
  {
    impl_.reset();
  }

private:
  struct Concept
  {
    virtual ~Concept() = default;
    virtual size_t subscriptionCount() const = 0;
    virtual std::string topic() const = 0;
  };

  template <class PubT>
  struct Model : Concept
  {
    explicit Model(std::shared_ptr<PubT> pub) : pub_(std::move(pub))
    {
    }
    size_t subscriptionCount() const override
    {
      return pub_->get_subscription_count();
    }
    std::string topic() const override
    {
      return pub_->get_topic_name();
    }
    std::shared_ptr<PubT> pub_;
  };

  std::shared_ptr<Concept> impl_;
};

class Subscriber
{
public:
  Subscriber() = default;

  Subscriber(std::shared_ptr<void> sub, std::string topic) : sub_(std::move(sub)), topic_(std::move(topic))
  {
  }

  std::string getTopic() const
  {
    return topic_;
  }

  void shutdown()
  {
    sub_.reset();
  }
  explicit operator bool() const
  {
    return static_cast<bool>(sub_);
  }

private:
  std::shared_ptr<void> sub_;
  std::string topic_;
};

class ServiceServer
{
public:
  ServiceServer() = default;
  explicit ServiceServer(std::shared_ptr<void> srv) : srv_(std::move(srv))
  {
  }
  void shutdown()
  {
    srv_.reset();
  }
  explicit operator bool() const
  {
    return static_cast<bool>(srv_);
  }

private:
  std::shared_ptr<void> srv_;
};

/**
 * Blocking service client, as roscpp's ServiceClient is.
 *
 * The blocking part is the problem. rclcpp has no synchronous call: spinning a
 * node from inside one of its own callbacks deadlocks, because the executor is
 * already inside that callback and will never reach the response. roscpp had no
 * such restriction, and the call sites here - the visual odometry reset, for one
 * - do exactly that.
 *
 * So the client gets its own node and its own executor, separate from the one
 * running the caller. The call then blocks the calling thread without touching
 * the executor it is running under.
 *
 * The timeout is what roscpp did not have: a service that never answers used to
 * hang forever, and here it gives up and reports failure instead.
 */
template <class S>
class ServiceClient
{
public:
  ServiceClient() = default;

  ServiceClient(std::shared_ptr<rclcpp::Node> owner, const std::string& name) : name_(name)
  {
    node_ =
        std::make_shared<rclcpp::Node>("ros_compat_service_client_" + std::to_string(reinterpret_cast<uintptr_t>(this)),
                                       owner ? owner->get_namespace() : "");
    client_ = node_->create_client<S>(name);
    executor_.add_node(node_);
  }

  bool exists(double timeout_sec = 0.0)
  {
    return client_->wait_for_service(std::chrono::duration<double>(timeout_sec));
  }

  bool call(typename S::Request& req, typename S::Response& res, double timeout_sec = 5.0)
  {
    if (!client_->service_is_ready())
      return false;
    auto request = std::make_shared<typename S::Request>(req);
    auto future = client_->async_send_request(request);
    if (executor_.spin_until_future_complete(future, std::chrono::duration<double>(timeout_sec)) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }
    res = *future.get();
    return true;
  }

private:
  std::string name_;
  std::shared_ptr<rclcpp::Node> node_;
  typename rclcpp::Client<S>::SharedPtr client_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

class Timer
{
public:
  Timer() = default;
  explicit Timer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer))
  {
  }

  void start()
  {
    if (timer_)
      timer_->reset();
  }
  void stop()
  {
    if (timer_)
      timer_->cancel();
  }
  bool hasStarted() const
  {
    return timer_ && !timer_->is_canceled();
  }
  explicit operator bool() const
  {
    return static_cast<bool>(timer_);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

/** roscpp hands a TimerEvent to timer callbacks; ROS2 passes nothing. */
struct TimerEvent
{
  Time current_expected{ 0, 0, RCL_ROS_TIME };
  Time current_real{ 0, 0, RCL_ROS_TIME };
  Time last_expected{ 0, 0, RCL_ROS_TIME };
  Time last_real{ 0, 0, RCL_ROS_TIME };
};

class NodeHandle
{
public:
  /**
   * Default handle onto the process-global node.
   *
   * roscpp's `ros::NodeHandle nh;` addresses the process's own node and, for
   * parameters, the global parameter server. ROS2 has no global server, so the
   * closest faithful mapping is the node registered with setGlobalNode(). Code
   * that default-constructs a handle to read a parameter - RobotModel does this
   * in two places - then keeps working without threading a node through every
   * constructor.
   */
  NodeHandle() : node_(detail::globalNode())
  {
  }

  /** Root handle onto a node. */
  explicit NodeHandle(std::shared_ptr<rclcpp::Node> node, const std::string& ns = "") : node_(std::move(node)), ns_(ns)
  {
  }

  /** Child handle: `ros::NodeHandle(parent, "sub_ns")`. */
  NodeHandle(const NodeHandle& parent, const std::string& ns)
    : node_(parent.node_), ns_(detail::join(parent.ns_, ns, '/')), private_(parent.private_)
  {
  }

  std::shared_ptr<rclcpp::Node> node() const
  {
    return node_;
  }
  const std::string& getNamespace() const
  {
    return ns_;
  }

  /** Mark this handle as the roscpp "~" private handle. */
  NodeHandle privateHandle() const
  {
    NodeHandle h(*this);
    h.private_ = true;
    return h;
  }

  std::string resolveName(const std::string& name) const
  {
    std::string resolved = detail::join(ns_, name, '/');
    if (private_ && !resolved.empty() && resolved.front() != '/')
      resolved = "~/" + resolved;
    return resolved;
  }

  // ---- publish / subscribe -------------------------------------------------

  template <class M>
  Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
  {
    rclcpp::QoS qos(rclcpp::KeepLast(queue_size ? queue_size : 1));
    if (latch)
      qos.transient_local();
    return Publisher(node_->create_publisher<M>(resolveName(topic), qos));
  }

  // Spelled std::shared_ptr<const M> rather than ConstPtr<M>: the latter is
  // `typename M::ConstSharedPtr`, a non-deduced context, so M could not be
  // deduced from the callback and every call site would have to name the
  // message type explicitly. roscpp's own signature has the same shape.
  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(const std::shared_ptr<const M>&),
                       T* obj, const TransportHints& = TransportHints())
  {
    auto cb = [obj, fp](const std::shared_ptr<const M>& msg) { (obj->*fp)(msg); };
    const std::string resolved = resolveName(topic);
    return Subscriber(
        node_->create_subscription<M>(resolved, rclcpp::QoS(rclcpp::KeepLast(queue_size ? queue_size : 1)), cb),
        resolved);
  }

  /** Overload for callbacks taking the message by value, as several here do. */
  template <class M, class T>
  Subscriber subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(const M&), T* obj,
                       const TransportHints& = TransportHints())
  {
    auto cb = [obj, fp](const ConstPtr<M>& msg) { (obj->*fp)(*msg); };
    const std::string resolved = resolveName(topic);
    return Subscriber(
        node_->create_subscription<M>(resolved, rclcpp::QoS(rclcpp::KeepLast(queue_size ? queue_size : 1)), cb),
        resolved);
  }

  // ---- services ------------------------------------------------------------

  template <class S, class T>
  ServiceServer advertiseService(const std::string& name, bool (T::*fp)(typename S::Request&, typename S::Response&),
                                 T* obj)
  {
    auto cb = [obj, fp](const std::shared_ptr<typename S::Request> req, std::shared_ptr<typename S::Response> res) {
      (obj->*fp)(*req, *res);
    };
    return ServiceServer(node_->create_service<S>(resolveName(name), cb));
  }

  // ---- timers --------------------------------------------------------------

  template <class T>
  Timer createTimer(Duration period, void (T::*fp)(const TimerEvent&), T* obj, bool oneshot = false,
                    bool autostart = true)
  {
    auto cb = [obj, fp]() {
      TimerEvent ev;
      (obj->*fp)(ev);
    };
    auto timer = node_->create_wall_timer(std::chrono::nanoseconds(period.nanoseconds()), cb);
    if (oneshot || !autostart)
      timer->cancel();
    return Timer(timer);
  }

  // ---- parameters ----------------------------------------------------------

  bool hasParam(const std::string& name) const
  {
    return node_ && node_->has_parameter(detail::toParamName(ns_, name));
  }

  template <class T>
  bool getParam(const std::string& name, T& value) const
  {
    const std::string full = detail::toParamName(ns_, name);
    if (!node_ || !node_->has_parameter(full))
      return false;
    return node_->get_parameter(full, value);
  }

  template <class T>
  bool param(const std::string& name, T& value, const T& default_value) const
  {
    if (getParam(name, value))
      return true;
    value = default_value;
    return false;
  }

  template <class T>
  T param(const std::string& name, const T& default_value) const
  {
    T value;
    param(name, value, default_value);
    return value;
  }

  template <class T>
  void setParam(const std::string& name, const T& value)
  {
    if (!node_)
      return;
    const std::string full = detail::toParamName(ns_, name);
    if (!node_->has_parameter(full))
      node_->declare_parameter(full, rclcpp::ParameterValue(value));
    node_->set_parameter(rclcpp::Parameter(full, value));
  }

  explicit operator bool() const
  {
    return static_cast<bool>(node_);
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string ns_;
  bool private_ = false;
};

// ---- process-level helpers -------------------------------------------------

inline void init(int& argc, char** argv, const std::string&)
{
  rclcpp::init(argc, argv);
}

/**
 * Create a node wired the way this compat layer needs.
 *
 * automatically_declare_parameters_from_overrides is what makes hasParam()
 * honest: without it, YAML-supplied parameters are invisible to has_parameter()
 * and every `if (hasParam(...))` branch in the stack quietly takes its default.
 */
inline std::shared_ptr<rclcpp::Node> createNode(const std::string& name, const std::string& ns = "")
{
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  options.allow_undeclared_parameters(true);
  return std::make_shared<rclcpp::Node>(name, ns, options);
}

inline void setGlobalNode(const NodeHandle& nh)
{
  detail::globalNode() = nh.node();
}

inline bool ok()
{
  return rclcpp::ok();
}
inline void shutdown()
{
  rclcpp::shutdown();
}

inline void spin()
{
  if (detail::globalNode())
    rclcpp::spin(detail::globalNode());
}

inline void spinOnce()
{
  if (detail::globalNode())
    rclcpp::spin_some(detail::globalNode());
}

/**
 * Current ROS time.
 *
 * Requires setGlobalNode() to have been called, otherwise there is no clock to
 * read and this returns zero. Under simulation the node must also have
 * use_sim_time set, or the timeout logic in the estimators drifts against the
 * MuJoCo clock.
 */
inline Time now()
{
  if (auto node = detail::globalNode())
    return node->now();
  return Time(0, 0, RCL_ROS_TIME);
}

}  // namespace ros_compat

// Logging. rclcpp's macros need a logger, which the roscpp ones do not have, so
// these resolve one from the global node and fall back to a named logger before
// the node exists.
#define ROS_COMPAT_LOGGER()                                                                                            \
  (::ros_compat::detail::globalNode() ? ::ros_compat::detail::globalNode()->get_logger() :                             \
                                        rclcpp::get_logger("aerial_robot"))

#define ROS_COMPAT_DEBUG(...) RCLCPP_DEBUG(ROS_COMPAT_LOGGER(), __VA_ARGS__)
#define ROS_COMPAT_INFO(...) RCLCPP_INFO(ROS_COMPAT_LOGGER(), __VA_ARGS__)
#define ROS_COMPAT_WARN(...) RCLCPP_WARN(ROS_COMPAT_LOGGER(), __VA_ARGS__)
#define ROS_COMPAT_ERROR(...) RCLCPP_ERROR(ROS_COMPAT_LOGGER(), __VA_ARGS__)
#define ROS_COMPAT_FATAL(...) RCLCPP_FATAL(ROS_COMPAT_LOGGER(), __VA_ARGS__)

#define ROS_COMPAT_DEBUG_STREAM(args) RCLCPP_DEBUG_STREAM(ROS_COMPAT_LOGGER(), args)
#define ROS_COMPAT_INFO_STREAM(args) RCLCPP_INFO_STREAM(ROS_COMPAT_LOGGER(), args)
#define ROS_COMPAT_WARN_STREAM(args) RCLCPP_WARN_STREAM(ROS_COMPAT_LOGGER(), args)
#define ROS_COMPAT_ERROR_STREAM(args) RCLCPP_ERROR_STREAM(ROS_COMPAT_LOGGER(), args)
#define ROS_COMPAT_FATAL_STREAM(args) RCLCPP_FATAL_STREAM(ROS_COMPAT_LOGGER(), args)

// roscpp throttles in seconds, rclcpp in milliseconds.
#define ROS_COMPAT_INFO_THROTTLE(p, ...)                                                                               \
  RCLCPP_INFO_THROTTLE(ROS_COMPAT_LOGGER(), *::ros_compat::detail::globalNode()->get_clock(),                          \
                       static_cast<int>((p) * 1000), __VA_ARGS__)
#define ROS_COMPAT_WARN_THROTTLE(p, ...)                                                                               \
  RCLCPP_WARN_THROTTLE(ROS_COMPAT_LOGGER(), *::ros_compat::detail::globalNode()->get_clock(),                          \
                       static_cast<int>((p) * 1000), __VA_ARGS__)
#define ROS_COMPAT_ERROR_THROTTLE(p, ...)                                                                              \
  RCLCPP_ERROR_THROTTLE(ROS_COMPAT_LOGGER(), *::ros_compat::detail::globalNode()->get_clock(),                         \
                        static_cast<int>((p) * 1000), __VA_ARGS__)

#define ROS_COMPAT_INFO_ONCE(...) RCLCPP_INFO_ONCE(ROS_COMPAT_LOGGER(), __VA_ARGS__)
#define ROS_COMPAT_WARN_ONCE(...) RCLCPP_WARN_ONCE(ROS_COMPAT_LOGGER(), __VA_ARGS__)
#define ROS_COMPAT_ERROR_ONCE(...) RCLCPP_ERROR_ONCE(ROS_COMPAT_LOGGER(), __VA_ARGS__)

#define ROS_COMPAT_INFO_STREAM_ONCE(args) RCLCPP_INFO_STREAM_ONCE(ROS_COMPAT_LOGGER(), args)
#define ROS_COMPAT_WARN_STREAM_ONCE(args) RCLCPP_WARN_STREAM_ONCE(ROS_COMPAT_LOGGER(), args)
#define ROS_COMPAT_ERROR_STREAM_ONCE(args) RCLCPP_ERROR_STREAM_ONCE(ROS_COMPAT_LOGGER(), args)

// rclcpp has no _COND form; the condition is spelled out at the call site.
#define ROS_COMPAT_INFO_COND(cond, ...)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      RCLCPP_INFO(ROS_COMPAT_LOGGER(), __VA_ARGS__);                                                                   \
  } while (0)
#define ROS_COMPAT_WARN_COND(cond, ...)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      RCLCPP_WARN(ROS_COMPAT_LOGGER(), __VA_ARGS__);                                                                   \
  } while (0)
#define ROS_COMPAT_ERROR_COND(cond, ...)                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    if (cond)                                                                                                          \
      RCLCPP_ERROR(ROS_COMPAT_LOGGER(), __VA_ARGS__);                                                                  \
  } while (0)
