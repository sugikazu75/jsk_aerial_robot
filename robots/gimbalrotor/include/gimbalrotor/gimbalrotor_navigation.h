// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/flight_navigation.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>
#include <spinal/DesireCoord.h>

namespace aerial_robot_navigation
{
class GimbalrotorNavigator : public BaseNavigator
{
public:
  GimbalrotorNavigator();
  ~GimbalrotorNavigator()
  {
  }

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du) override;

  void update() override;
  bool isInflightState() override;

  // landing: first the body level (the yaw kept) and the joints to landing_joint_angles, holding the
  // height, then the descent (as DragonNavigator)
  static constexpr uint8_t PRE_LAND_STATE = 0x20;

private:
  ros::Publisher target_baselink_rpy_pub_;
  ros::Subscriber final_target_baselink_rot_sub_, final_target_baselink_rpy_sub_;

  void baselinkRotationProcess();
  void rosParamInit() override;
  void targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg);
  void targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
  void naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg) override;

  void reset() override;
  void landingProcess();

  /* target baselink rotation */
  double prev_rotation_stamp_;
  tf::Quaternion curr_target_baselink_rot_, final_target_baselink_rot_;
  bool eq_cog_world_;

  /* rosparam */
  double baselink_rot_change_thresh_;
  double baselink_rot_pub_interval_;

  /* landing */
  bool level_flag_ = false;
  ros::Publisher landing_joint_pub_;
  sensor_msgs::JointState landing_joint_msg_;  // names and angles of navigation/landing_joint_*
  double landing_joint_stamp_ = 0;
  double landing_joint_tolerance_;
};
};  // namespace aerial_robot_navigation
