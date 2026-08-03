// Compiles the ros_compat API surface that the aerial robot stack actually
// uses, against whichever ROS version is being built. This is a compile-time
// check: if the ROS1 and ROS2 backings drift apart, this stops building.
//
// The patterns exercised here were taken from the real call sites - child
// NodeHandles used as parameter namespaces, getParam with defaults, latched
// publishers, member-function subscribe callbacks, advertiseService and
// createTimer.

#include <aerial_robot_ros_compat/message.h>
#include <aerial_robot_ros_compat/ros_compat.h>

#if AERIAL_ROBOT_ROS_VERSION == 1
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#else
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#endif

AERIAL_ROBOT_MSG_NAMESPACE(geometry_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(sensor_msgs);
AERIAL_ROBOT_SRV_NAMESPACE(std_srvs);

namespace
{

using PoseMsg = geometry_msgs_c::Vector3Stamped;
using JointMsg = sensor_msgs_c::JointState;
using SetBool = std_srvs_s::SetBool;

class Exerciser
{
public:
  void initialize(ros_compat::NodeHandle nh, ros_compat::NodeHandle nhp)
  {
    nh_ = nh;
    nhp_ = nhp;

    // child handles used as namespaces, the dominant idiom in this stack
    ros_compat::NodeHandle control_nh(nh_, "controller");
    ros_compat::NodeHandle motor_nh(nh_, "motor_info");

    ros_compat::getParam<double>(motor_nh, "max_pwm", max_pwm_, 0.0);
    ros_compat::getParam<int>(nh_, "uav_model", uav_model_, 0);
    ros_compat::getParam<bool>(nhp_, "param_verbose", verbose_, false);
    ros_compat::getParam<std::string>(control_nh, "mode", mode_, std::string("pid"));

    if (control_nh.hasParam("control_verbose"))
      control_nh.getParam("control_verbose", verbose_);

    pub_ = nh_.advertise<PoseMsg>("debug/pose", 10);
    latched_pub_ = nh_.advertise<JointMsg>("joint_states", 1, true);

    sub_ = nh_.subscribe<PoseMsg>("target", 1, &Exerciser::poseCallback, this);

    srv_ = ros_compat::advertiseService<SetBool>(nh_, "estimate_flag", &Exerciser::setFlag, this);

    timer_ = nh_.createTimer(ros_compat::duration(1.0 / 40.0), &Exerciser::update, this);

    stamp_ = ros_compat::now();
  }

private:
  void poseCallback(const ros_compat::ConstPtr<PoseMsg>& msg)
  {
    PoseMsg out = *msg;
    out.header.stamp = ros_compat::now();
    pub_.publish(out);
    ROS_COMPAT_INFO("got pose, %zu subscribers", pub_.getNumSubscribers());
  }

  bool setFlag(SetBool::Request& req, SetBool::Response& res)
  {
    flag_ = req.data;
    res.success = true;
    return true;
  }

  void update(const ros_compat::TimerEvent&)
  {
    if (ros_compat::toSec(ros_compat::now() - stamp_) > 1.0)
      ROS_COMPAT_WARN("stale");
    ROS_COMPAT_DEBUG_STREAM("mode " << mode_ << " model " << uav_model_ << " pwm " << max_pwm_);
  }

  ros_compat::NodeHandle nh_, nhp_;
  ros_compat::Publisher pub_, latched_pub_;
  ros_compat::Subscriber sub_;
  ros_compat::ServiceServer srv_;
  ros_compat::Timer timer_;
  ros_compat::Time stamp_;
  double max_pwm_ = 0.0;
  int uav_model_ = 0;
  bool verbose_ = false, flag_ = false;
  std::string mode_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros_compat::NodeHandle nh = ros_compat::initNode(argc, argv, "ros_compat_compile_test");

  Exerciser exerciser;
  exerciser.initialize(nh, ros_compat::privateNodeHandle(nh));

  ros_compat::shutdown();
  return 0;
}
