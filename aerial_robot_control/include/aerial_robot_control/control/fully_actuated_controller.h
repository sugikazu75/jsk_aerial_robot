// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once

#include <aerial_robot_ros_compat/message.h>
#include <aerial_robot_ros_compat/ros_compat.h>
#include <aerial_robot_control/control/base/pose_linear_controller.h>
#if AERIAL_ROBOT_ROS_VERSION == 1
#  include <aerial_robot_msgs/WrenchAllocationMatrix.h>
#  include <spinal/FourAxisCommand.h>
#  include <spinal/RollPitchYawTerms.h>
#  include <spinal/TorqueAllocationMatrixInv.h>
#else
#  include <aerial_robot_msgs/msg/wrench_allocation_matrix.hpp>
#  include <spinal/msg/four_axis_command.hpp>
#  include <spinal/msg/roll_pitch_yaw_terms.hpp>
#  include <spinal/msg/torque_allocation_matrix_inv.hpp>
#endif
AERIAL_ROBOT_MSG_NAMESPACE(aerial_robot_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(spinal);

using boost::algorithm::clamp;

namespace aerial_robot_control
{
  class FullyActuatedController: public PoseLinearController
  {
  public:
    FullyActuatedController();
    virtual ~FullyActuatedController() = default;

    void initialize(ros_compat::NodeHandle nh, ros_compat::NodeHandle nhp,
                    ros_compat::SharedPtr<aerial_robot_model::RobotModel> robot_model,
                    ros_compat::SharedPtr<aerial_robot_estimation::StateEstimator> estimator,
                    ros_compat::SharedPtr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;

    virtual void reset() override;

    virtual void controlCore() override;
    virtual void sendCmd() override;

  private:
    ros_compat::Publisher flight_cmd_pub_; //for spinal
    ros_compat::Publisher rpy_gain_pub_; //for spinal
    ros_compat::Publisher torque_allocation_matrix_inv_pub_; //for spinal
    double torque_allocation_matrix_inv_pub_stamp_;
    ros_compat::Publisher wrench_allocation_matrix_pub_; //for debug
    ros_compat::Publisher wrench_allocation_matrix_inv_pub_; //for debug
    double wrench_allocation_matrix_pub_stamp_;

    Eigen::MatrixXd q_mat_;
    Eigen::MatrixXd q_mat_inv_;

    std::vector<float> target_base_thrust_;
    double candidate_yaw_term_;

    double torque_allocation_matrix_inv_pub_interval_;
    double wrench_allocation_matrix_pub_interval_;

    void setAttitudeGains();
    void rosParamInit();

    void sendFourAxisCommand();
    void sendTorqueAllocationMatrixInv();


  };
} //namespace aerial_robot_control
