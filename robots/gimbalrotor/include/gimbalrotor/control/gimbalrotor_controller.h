// -*- mode: c++ -*-

#pragma once

#include <numeric>
#include <aerial_robot_ros_compat/message.h>
#include <aerial_robot_ros_compat/ros_compat.h>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <aerial_robot_control/control/fully_actuated_controller.h>
#include <aerial_robot_estimation/state_estimation.h>
#if AERIAL_ROBOT_ROS_VERSION == 1
#  include <sensor_msgs/JointState.h>
#  include <spinal/FourAxisCommand.h>
#  include <spinal/RollPitchYawTerms.h>
#  include <spinal/TorqueAllocationMatrixInv.h>
#  include <std_msgs/Float32MultiArray.h>
#  include <std_msgs/UInt8.h>
#else
#  include <sensor_msgs/msg/joint_state.hpp>
#  include <spinal/msg/four_axis_command.hpp>
#  include <spinal/msg/roll_pitch_yaw_terms.hpp>
#  include <spinal/msg/torque_allocation_matrix_inv.hpp>
#  include <std_msgs/msg/float32_multi_array.hpp>
// rosidl's snake_case of UInt8, not uint8.hpp.
#  include <std_msgs/msg/u_int8.hpp>
#endif
AERIAL_ROBOT_MSG_NAMESPACE(sensor_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(spinal);
AERIAL_ROBOT_MSG_NAMESPACE(std_msgs);
#include <gimbalrotor/model/gimbalrotor_robot_model.h>

namespace aerial_robot_control
{
class GimbalrotorController : public PoseLinearController
{
public:
  GimbalrotorController();
  ~GimbalrotorController() = default;

  void initialize(ros_compat::NodeHandle nh, ros_compat::NodeHandle nhp,
                  ros_compat::SharedPtr<aerial_robot_model::RobotModel> robot_model,
                  ros_compat::SharedPtr<aerial_robot_estimation::StateEstimator> estimator,
                  ros_compat::SharedPtr<aerial_robot_navigation::BaseNavigator> navigator,
                  double ctrl_loop_rate) override;

private:
  ros_compat::Publisher flight_cmd_pub_;
  ros_compat::Publisher gimbal_control_pub_;
  ros_compat::Publisher gimbal_state_pub_;
  ros_compat::Publisher target_vectoring_force_pub_;
  ros_compat::Publisher rpy_gain_pub_;                      // for spinal
  ros_compat::Publisher torque_allocation_matrix_inv_pub_;  // for spinal
  ros_compat::Publisher gimbal_dof_pub_;                    // for spinal

  ros_compat::SharedPtr<GimbalrotorRobotModel> gimbalrotor_robot_model_;
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

  void rosParamInit();
  bool update() override;
  virtual void reset() override;
  void controlCore() override;
  void sendCmd() override;
  void sendFourAxisCommand();
  void sendGimbalCommand();
  void sendTorqueAllocationMatrixInv();
  void setAttitudeGains();
};
};  // namespace aerial_robot_control
