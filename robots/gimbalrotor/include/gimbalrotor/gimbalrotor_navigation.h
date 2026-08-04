// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_ros_compat/message.h>
#include <aerial_robot_ros_compat/ros_compat.h>
#include <aerial_robot_control/flight_navigation.h>
#if AERIAL_ROBOT_ROS_VERSION == 1
#  include <geometry_msgs/QuaternionStamped.h>
#  include <geometry_msgs/Vector3Stamped.h>
#  include <spinal/DesireCoord.h>
#else
#  include <geometry_msgs/msg/quaternion_stamped.hpp>
#  include <geometry_msgs/msg/vector3_stamped.hpp>
#  include <spinal/msg/desire_coord.hpp>
#endif
AERIAL_ROBOT_MSG_NAMESPACE(geometry_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(spinal);

namespace aerial_robot_navigation
{
class GimbalrotorNavigator : public BaseNavigator
{
public:
  GimbalrotorNavigator();
  ~GimbalrotorNavigator()
  {
  }

  void initialize(ros_compat::NodeHandle nh, ros_compat::NodeHandle nhp,
                  ros_compat::SharedPtr<aerial_robot_model::RobotModel> robot_model,
                  ros_compat::SharedPtr<aerial_robot_estimation::StateEstimator> estimator,
                  double loop_du) override;

  void update() override;

private:
  ros_compat::Publisher target_baselink_rpy_pub_;
  ros_compat::Subscriber final_target_baselink_rot_sub_, final_target_baselink_rpy_sub_;

  void baselinkRotationProcess();
  void rosParamInit() override;
  void targetBaselinkRotCallback(const ros_compat::ConstPtr<geometry_msgs_c::QuaternionStamped>& msg);
  void targetBaselinkRPYCallback(const ros_compat::ConstPtr<geometry_msgs_c::Vector3Stamped>& msg);
  void naviCallback(const ros_compat::ConstPtr<aerial_robot_msgs_c::FlightNav>& msg) override;

  void reset() override;

  /* target baselink rotation */
  double prev_rotation_stamp_;
  tf2::Quaternion curr_target_baselink_rot_, final_target_baselink_rot_;
  bool eq_cog_world_;

  /* rosparam */
  double baselink_rot_change_thresh_;
  double baselink_rot_pub_interval_;
};
};  // namespace aerial_robot_navigation
