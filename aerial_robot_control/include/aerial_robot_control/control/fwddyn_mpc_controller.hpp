// -*- mode: c++ -*-
#pragma once

#include <pinocchio/fwd.hpp>

#include <aerial_robot_control/control/base/base.h>
#include <aerial_robot_control/control/fwddyn_mpc_control_problem.hpp>
#include <aerial_robot_dynamics/robot_model.h>
#include <sensor_msgs/JointState.h>
#include <spinal/FourAxisCommand.h>
#include <tf/transform_broadcaster.h>

#include <map>

namespace aerial_robot_control
{

class FwddynMpcController : public ControlBase
{
public:
  FwddynMpcController();
  virtual ~FwddynMpcController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

  void activate() override;
  bool update() override;
  void reset() override;

protected:
  void controlCore();
  void sendCmd();

private:
  ros::Publisher four_axis_command_pub_;
  ros::Publisher joints_ctrl_pub_;
  ros::Publisher target_cog_pos_pub_;
  ros::Subscriber joint_state_sub_;
  tf::TransformBroadcaster optimized_root_tf_broadcaster_;

  sensor_msgs::JointState joint_state_;
  bool joint_state_received_;

  Eigen::VectorXd curr_q_;
  Eigen::VectorXd curr_dq_;
  Eigen::VectorXd curr_tau_;

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pin_model_;
  pinocchio::FrameIndex baselink_frame_id_;

  std::map<std::string, int> joint_name_to_q_idx_;
  std::map<std::string, int> joint_name_to_v_idx_;
  int n_joints_;  // non-floating-base DOF

  FwddynMpcControlProblem mpc_problem_;

  // MPC tuning parameters
  int num_mpc_nodes_;
  int mpc_max_iter_;
  int max_init_iter_;
  double mpc_dt_;
  double mpc_elapsed_time_;

  // cost weights
  Eigen::Vector3d com_track_weight_;
  Eigen::VectorXd x_state_weight_;
  double control_weight_;
  double thrust_barrier_weight_;
  Eigen::VectorXd thrust_lb_;
  Eigen::VectorXd thrust_ub_;

  // reference state (nq + nv) used for state regularisation
  Eigen::VectorXd x_ref_;

  // for debugging
  double target_cog_pos_pub_duration_ = 1.0;
  double last_target_cog_pos_pub_time_ = 0.0;
  double tf_broadcast_duration_ = 0.1;
  double last_tf_broadcast_time_ = 0.0;

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  Eigen::VectorXd buildCurrentState();
  void broadcastOptimizedRootTransforms();
  void publishJointsCtrl();
};

}  // namespace aerial_robot_control
