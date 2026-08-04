// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, JSK Lab
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

#include <aerial_robot_estimation/sensor/imu.h>
#if AERIAL_ROBOT_ROS_VERSION == 1
#  include <geometry_msgs/Vector3Stamped.h>
#else
#  include <geometry_msgs/msg/vector3_stamped.hpp>
#endif
AERIAL_ROBOT_MSG_NAMESPACE(geometry_msgs);
#include <mutex>

using namespace Eigen;
using namespace std;

namespace sensor_plugin
{
  class DragonImu :public sensor_plugin::Imu
  {
  public:

    void initialize(ros_compat::NodeHandle nh,
                    ros_compat::SharedPtr<aerial_robot_model::RobotModel> robot_model,
                    ros_compat::SharedPtr<aerial_robot_estimation::StateEstimator> estimator,
                    string sensor_name, int index) override;

    void setFilteredOmegaCog(const tf2::Vector3 filtered_omega_cog)
    {
      std::lock_guard<std::mutex> lock(omega_mutex_);
      filtered_omega_cog_ = filtered_omega_cog;
    }

    void setFilteredVelCog(const tf2::Vector3 filtered_vel_cog)
    {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      filtered_vel_cog_ = filtered_vel_cog;
    }

    const tf2::Vector3 getFilteredOmegaCog()
    {
      std::lock_guard<std::mutex> lock(omega_mutex_);
      return filtered_omega_cog_;
    }

    const tf2::Vector3 getFilteredVelCog()
    {
      std::lock_guard<std::mutex> lock(vel_mutex_);
      return filtered_vel_cog_;
    }

  protected:

    void ImuCallback(const ros_compat::ConstPtr<spinal_c::Imu>& imu_msg) override;

    // work around to obtain filter states
    std::mutex omega_mutex_;
    std::mutex vel_mutex_;
    tf2::Vector3 filtered_vel_cog_;
    tf2::Vector3 filtered_omega_cog_;
    IirFilter lpf_omega_; // for gyro

    ros_compat::Publisher omega_filter_pub_; // debug
  };
};





