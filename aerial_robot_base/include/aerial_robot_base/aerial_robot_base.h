#pragma once

#include <aerial_robot_ros_compat/ros_compat.h>
#if AERIAL_ROBOT_ROS_VERSION == 1
#  include <ros/callback_queue.h>
#endif
#include <pluginlib/class_loader.hpp>
#include <aerial_robot_control/control/base/base.h>
#include <aerial_robot_control/flight_navigation.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_model/model/aerial_robot_model_ros.h>

using namespace std;

class AerialRobotBase
{
 public:
  AerialRobotBase(ros_compat::NodeHandle nh, ros_compat::NodeHandle nh_private);
  ~AerialRobotBase();

  void mainFunc(const ros_compat::TimerEvent & e);

 private:
  ros_compat::NodeHandle nh_;
  ros_compat::NodeHandle nhp_;
  ros_compat::Timer main_timer_;

  ros_compat::SharedPtr<aerial_robot_model::RobotModelRos> robot_model_ros_;
  ros_compat::SharedPtr<aerial_robot_estimation::StateEstimator>  estimator_;

  pluginlib::ClassLoader<aerial_robot_navigation::BaseNavigator> navigator_loader_;
  ros_compat::SharedPtr<aerial_robot_navigation::BaseNavigator> navigator_;

  pluginlib::ClassLoader<aerial_robot_control::ControlBase> controller_loader_;
  ros_compat::SharedPtr<aerial_robot_control::ControlBase> controller_;

#if AERIAL_ROBOT_ROS_VERSION == 1
  ros_compat::AsyncSpinner callback_spinner_; // Use 4 threads
  ros_compat::AsyncSpinner main_loop_spinner_; // Use 1 threads
  ros_compat::CallbackQueue main_loop_queue_;
#endif
};
