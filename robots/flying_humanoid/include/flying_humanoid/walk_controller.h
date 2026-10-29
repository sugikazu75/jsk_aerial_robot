// -*- mode: c++ -*-
#pragma once

#include <flying_humanoid/robot_model.h>

#include <pinocchio/multibody/data.hpp>

#include <aerial_robot_control/control/base/base.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <Eigen/Core>

#include <map>
#include <mutex>
#include <string>
#include <vector>

namespace flying_humanoid
{
// Replays the walk trajectory planned by walk_planner.
//  - "walk/trajectory" (std_msgs/String): path of a new trajectory CSV. It is loaded unless a replay is running,
//    and "walk/trajectory_loaded" echoes the path back.
//  - "walk/start" (std_msgs/Bool): true starts the replay, false rewinds it.
// Before the replay, the robot keeps the first sample of the trajectory. The trajectory starts from the state measured
// when it was planned, so plan it just before walking.
// The thrusts are fed forward from the trajectory, plus a PD feedback on the attitude of the baselink.
class WalkController : public aerial_robot_control::ControlBase
{
public:
  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_du) override;
  bool update() override;
  void reset() override;

private:
  struct Sample
  {
    double time;
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::VectorXd thrust;
  };

  void trajectoryCallback(const std_msgs::String::ConstPtr& msg);
  void startCallback(const std_msgs::Bool::ConstPtr& msg);
  bool loadTrajectory(const std::string& filename);
  void loadPendingTrajectory();
  void rewind();

  void publishJointCommand(const Sample& sample);
  void publishDesireCoord(const Sample& sample);
  void publishThrust(const Sample& sample);
  Eigen::Matrix3d baselinkRotation(const Sample& sample);

  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<std::string> gimbal_names_;

  ros::Publisher joints_ctrl_pub_;
  ros::Publisher gimbals_ctrl_pub_;
  ros::Publisher four_axis_command_pub_;
  ros::Publisher desire_coord_pub_;
  ros::Publisher trajectory_loaded_pub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber start_sub_;

  std::mutex mutex_;  // the callbacks and update() run in different threads
  std::string pending_trajectory_;
  bool start_requested_ = false;

  std::vector<Sample> trajectory_;
  std::size_t index_ = 0;
  double replay_time_ = 0.0;
  bool replaying_ = false;
  bool first_run_ = true;

  // attitude feedback on the thrusts
  double roll_p_gain_ = 5.0, roll_d_gain_ = 0.5;
  double pitch_p_gain_ = 5.0, pitch_d_gain_ = 0.5;
  double yaw_p_gain_ = 1.0, yaw_d_gain_ = 0.1;
  double max_moment_ = 2.0;          // [Nm]
  double sr_inverse_lambda_ = 1e-3;  // damping of the singularity-robust inverse of the moment map
};
}  // namespace flying_humanoid
