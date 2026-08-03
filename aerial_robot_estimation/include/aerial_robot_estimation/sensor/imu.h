// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#include <aerial_robot_estimation/sensor/base_plugin.h>

#if AERIAL_ROBOT_ROS_VERSION == 1
#  include <aerial_robot_msgs/Acc.h>
#  include <geometry_msgs/Vector3.h>
#  include <sensor_msgs/Imu.h>
#  include <spinal/Imu.h>
#else
#  include <aerial_robot_msgs/msg/acc.hpp>
#  include <geometry_msgs/msg/vector3.hpp>
#  include <sensor_msgs/msg/imu.hpp>
#  include <spinal/msg/imu.hpp>
#endif
AERIAL_ROBOT_MSG_NAMESPACE(aerial_robot_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(geometry_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(sensor_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(spinal);


using namespace Eigen;
using namespace std;

namespace sensor_plugin
{
  class Imu :public sensor_plugin::SensorBase
  {
  public:
    virtual void initialize(ros_compat::NodeHandle nh,
                            ros_compat::SharedPtr<aerial_robot_model::RobotModel> robot_model,
                            ros_compat::SharedPtr<aerial_robot_estimation::StateEstimator> estimator,
                            string sensor_name, int index) override;

    ~Imu() {}
    Imu();

    inline ros_compat::Time getStamp(){return imu_stamp_;}

  protected:
    ros_compat::Publisher  acc_pub_;
    ros_compat::Publisher  imu_pub_;
    ros_compat::Subscriber imu_sub_;

    /* rosparam */
    string imu_topic_name_;
    string imu_pub_topic_name_;

    int calib_count_;
    double acc_scale_, gyro_scale_, mag_scale_; /* the scale of sensor value */
    double level_acc_noise_sigma_, z_acc_noise_sigma_, level_acc_bias_noise_sigma_, z_acc_bias_noise_sigma_, angle_bias_noise_sigma_; /* sigma for kf */

    /* sensor internal */
    double sensor_dt_;

    /* imu */
    tf2::Vector3 omega_; /* the omega both of body frame */
    tf2::Vector3 mag_; /* the magnetometer of body frame */
    tf2::Vector3 acc_b_; /* the acceleration in baselink frame */
    tf2::Matrix3x3 raw_rot_; /* the raw rotation matrix from IMU */
    /* acc */
    std::array<tf2::Vector3, 2> acc_w_; /* the acceleration in world frame, for estimate_mode and expriment_mode */
    std::array<tf2::Vector3, 2> acc_non_bias_w_; /* the acceleration without bias in world frame for estimate_mode and expriment_mode */
    /* acc bias */
    tf2::Vector3 acc_bias_b_; /* the acceleration bias in baselink frame, only use z axis  */
    std::array<tf2::Vector3, 2> acc_bias_w_; /* the acceleration bias in world frame for estimate_mode and expriment_mode*/

    aerial_robot_msgs_c::States state_; /* for debug */

    double calib_time_;

    ros_compat::Time imu_stamp_;

    virtual void ImuCallback(const ros_compat::ConstPtr<spinal_c::Imu>& imu_msg);
    virtual void estimateProcess();
    void publishAccData();
    void publishRosImuData();
    void rosParamInit();
  };
};





