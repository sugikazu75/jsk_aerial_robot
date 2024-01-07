// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/base/base.h>
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <CasadiEigen/CasadiEigen.h>
#include <geometry_msgs/TransformStamped.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

namespace aerial_robot_control
{
  enum
    {
      X, Y, Z, ROLL, PITCH, YAW,
    };

  class DeltaController : public ControlBase
  {
  public:
    DeltaController();
    ~DeltaController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;
    bool update() override;
    void reset() override;

  private:
    ros::Publisher flight_cmd_pub_; //for spinal
    ros::Publisher rpy_gain_pub_; //for spinal
    ros::Publisher torque_allocation_matrix_inv_pub_; //for spinal
    std::string tf_prefix_;
    tf2_ros::TransformBroadcaster br_;

    std::thread visualize_thread_;

    double torque_allocation_matrix_inv_pub_stamp_;
    double torque_allocation_matrix_inv_pub_interval_;

    std::vector<float> target_base_thrust_;
    double target_roll_, target_pitch_;

    /* hyper parameters */
    double holizon_;
    int n_grid_;
    int nx_;
    int nu_;
    Eigen::MatrixXd x_weight_mat_;
    Eigen::MatrixXd u_weight_mat_;
    casadi::DM x_ref_;
    Eigen::VectorXd x_ref_eigen_;
    casadi::MX w_;
    casadi::MX g_;
    Eigen::VectorXd lbw_eigen_, ubw_eigen_;
    Eigen::VectorXd lbg_eigen_, ubg_eigen_;
    casadi::DM lbw_, ubw_;
    casadi::DM lbg_, ubg_;
    Eigen::VectorXd solution_;
    casadi::DM lam_x_, lam_g_;

    double dt_;
    double last_update_time_;
    double candidate_yaw_term_;
    double yaw_p_gain_;

    casadi::MX dynamics(casadi::MX x, casadi::MX u);
    void mpcInit();
    casadi::MX calcStageCost(casadi::MX x, casadi::MX u);
    casadi::MX calcTerminalCost(casadi::MX x);

    void rosParamInit();
    void controlCore();
    void sendCmd();
    void sendFourAxisCommand();
    void sendTorqueAllocationMatrixInv();
    void setAttitudeGains();
    void visualizeFunc();
  };
};
