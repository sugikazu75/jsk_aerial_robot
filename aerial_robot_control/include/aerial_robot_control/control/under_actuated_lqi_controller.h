// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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
#include <aerial_robot_control/control/under_actuated_controller.h>
#include <aerial_robot_control/control/utils/care.h>
#if AERIAL_ROBOT_ROS_VERSION == 1
#  include <aerial_robot_control/LQIConfig.h>
#  include <aerial_robot_msgs/FourAxisGain.h>
#  include <dynamic_reconfigure/server.h>
#  include <spinal/PMatrixPseudoInverseWithInertia.h>
#  include <spinal/RollPitchYawTerms.h>
#else
#  include <aerial_robot_msgs/msg/four_axis_gain.hpp>
#  include <spinal/msg/p_matrix_pseudo_inverse_with_inertia.hpp>
#  include <spinal/msg/roll_pitch_yaw_terms.hpp>
#endif
#include <thread>
AERIAL_ROBOT_MSG_NAMESPACE(aerial_robot_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(spinal);

namespace aerial_robot_control
{
  class UnderActuatedLQIController: public PoseLinearController
  {

  public:
    UnderActuatedLQIController();
    virtual ~UnderActuatedLQIController();

    void initialize(ros_compat::NodeHandle nh, ros_compat::NodeHandle nhp,
                    ros_compat::SharedPtr<aerial_robot_model::RobotModel> robot_model,
                    ros_compat::SharedPtr<aerial_robot_estimation::StateEstimator> estimator,
                    ros_compat::SharedPtr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

    void activate() override;

  protected:

    ros_compat::Publisher flight_cmd_pub_; // for spinal
    ros_compat::Publisher rpy_gain_pub_; // for spinal
    ros_compat::Publisher four_axis_gain_pub_;
    ros_compat::Publisher p_matrix_pseudo_inverse_inertia_pub_;

    bool verbose_;
#if AERIAL_ROBOT_ROS_VERSION == 1
    ros_compat::SharedPtr<dynamic_reconfigure::Server<aerial_robot_control::LQIConfig> > lqi_server_;
    dynamic_reconfigure::Server<aerial_robot_control::LQIConfig>::CallbackType dynamic_reconf_func_lqi_;
#endif

    double target_roll_, target_pitch_;
    double candidate_yaw_term_;
    std::vector<float> target_base_thrust_;

    int lqi_mode_;
    bool clamp_gain_;
    Eigen::MatrixXd K_;

    Eigen::Vector3d lqi_roll_pitch_weight_, lqi_yaw_weight_, lqi_z_weight_;
    std::vector<double> r_; // matrix R

    std::vector<Eigen::Vector3d> pitch_gains_, roll_gains_, yaw_gains_, z_gains_;

    bool gyro_moment_compensation_;

    bool realtime_update_;
    std::thread gain_generator_thread_;

    //private functions
    virtual bool checkRobotModel();
    void resetGain() { K_ = Eigen::MatrixXd(); }

    virtual void rosParamInit();
    virtual void controlCore() override;

    virtual bool optimalGain();
    virtual void clampGain();
    virtual void publishGain();

    virtual void sendCmd() override;
    virtual void sendFourAxisCommand();

    Eigen::MatrixXd getQInv();
    virtual void allocateYawTerm();
#if AERIAL_ROBOT_ROS_VERSION == 1
    void cfgLQICallback(aerial_robot_control::LQIConfig &config, uint32_t level); //dynamic reconfigure
#endif

    void sendRotationalInertiaComp();

    void gainGeneratorFunc();
  };
};
