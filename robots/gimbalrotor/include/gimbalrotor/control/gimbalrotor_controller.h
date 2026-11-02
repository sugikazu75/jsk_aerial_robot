// -*- mode: c++ -*-

#pragma once

#include <mutex>
#include <numeric>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <gimbalrotor/model/gimbalrotor_robot_model.h>

namespace aerial_robot_control
{
class GimbalrotorController : public PoseLinearController
{
public:
  GimbalrotorController();
  ~GimbalrotorController() = default;

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                  boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator, double ctrl_loop_rate) override;

private:
  ros::Publisher flight_cmd_pub_;
  ros::Publisher gimbal_control_pub_;
  ros::Publisher gimbal_state_pub_;
  ros::Publisher target_vectoring_force_pub_;
  ros::Publisher rpy_gain_pub_;                      // for spinal
  ros::Publisher torque_allocation_matrix_inv_pub_;  // for spinal
  ros::Publisher gimbal_dof_pub_;                    // for spinal
  ros::Publisher estimate_external_wrench_cog_pub_;  // both force and torque in CoG frame for visualization

  boost::shared_ptr<GimbalrotorRobotModel> gimbalrotor_robot_model_;
  std::vector<float> target_base_thrust_;
  std::vector<float> target_full_thrust_;
  std::vector<double> target_gimbal_angles_;
  bool hovering_approximate_;
  Eigen::VectorXd target_vectoring_f_;
  Eigen::VectorXd target_vectoring_f_trans_;
  Eigen::VectorXd target_vectoring_f_rot_;
  Eigen::MatrixXd integrated_map_inv_trans_;
  Eigen::MatrixXd integrated_map_inv_rot_;
  double candidate_yaw_term_;
  int gimbal_dof_;
  int rotor_coef_;
  bool gimbal_calc_in_fc_;
  bool underactuate_;
  double target_roll_ = 0.0, target_pitch_ = 0.0;

  /* feedforward compensation of the estimated external wrench */
  bool wrench_compensation_flag_;
  Eigen::VectorXd wrench_compensation_mask_;  // [fx, fy, fz (world frame), tx, ty, tz (CoG frame)]
  Eigen::VectorXd compensation_wrench_;       // masked estimated external wrench, updated in the estimation thread
  std::mutex compensation_mutex_;
  double wrench_compensation_start_delay_;    // wait for the convergence of the estimation after hovering [s]
  double wrench_compensation_average_duration_;  // average the estimation for the hand over of the I term [s]
  double wrench_compensation_force_std_thresh_;  // retry the averaging if the estimation fluctuates (e.g., touched) [N]
  double wrench_compensation_torque_std_thresh_;  // [Nm]
  Eigen::VectorXd compensation_sum_, compensation_sq_sum_;
  int compensation_sample_num_;
  bool compensation_active_;
  double hover_start_time_;
  Eigen::VectorXd applied_compensation_wrench_;
  std::string cog_frame_id_;

  void rosParamInit();
  bool update() override;
  virtual void reset() override;
  void controlCore() override;
  void externalWrenchEstimate() override;
  const Eigen::VectorXd getCompensationWrench();
  Eigen::VectorXd updateWrenchCompensation();
  void handOverIntegralTerm(const Eigen::VectorXd& wrench, bool to_compensation);
  void resetCompensationAverage();
  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
};
};  // namespace aerial_robot_control
