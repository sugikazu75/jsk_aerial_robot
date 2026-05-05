#pragma once

#include <pinocchio/fwd.hpp>  // should be included before any other pinocchio headers

#include <ros/ros.h>
#include <aerial_robot_dynamics/robot_model.h>

#include <memory>
#include <string>

namespace aerial_robot_dynamics
{
class PinocchioRobotModelRos
{
public:
  PinocchioRobotModelRos(ros::NodeHandle nh) : nh_(nh)
  {
    std::string robot_description;
    while (!nh_.getParam("robot_description", robot_description))
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Waiting for robot_description parameter...");
      ros::Duration(0.1).sleep();
    }

    std::string pinocchio_robot_description;
    while (!nh_.getParam("pinocchio_robot_description", pinocchio_robot_description))
    {
      ROS_ERROR_STREAM_THROTTLE(1.0, "Waiting for pinocchio_robot_description parameter...");
      ros::Duration(0.1).sleep();
    }

    ros::NodeHandle dynamics_nh(nh_, "dynamics");
    bool is_floating_base = true;
    double thrust_hessian_weight = 1.0;
    getParam<bool>(dynamics_nh, "is_floating_base", is_floating_base, true);
    getParam<double>(dynamics_nh, "thrust_hessian_weight", thrust_hessian_weight, 1.0);

    pinocchio_robot_model_ = std::make_shared<PinocchioRobotModel>(robot_description, pinocchio_robot_description,
                                                                   is_floating_base, thrust_hessian_weight);
  }
  ~PinocchioRobotModelRos() = default;

  std::shared_ptr<PinocchioRobotModel> getPinocchioRobotModel() const
  {
    return pinocchio_robot_model_;
  }

private:
  ros::NodeHandle nh_;
  std::shared_ptr<PinocchioRobotModel> pinocchio_robot_model_;

  template <class T>
  void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
  {
    nh.param<T>(param_name, param, default_value);
  }
};
}  // namespace aerial_robot_dynamics
