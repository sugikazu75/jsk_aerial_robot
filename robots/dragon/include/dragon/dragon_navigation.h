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

#include <aerial_robot_ros_compat/message.h>
#include <aerial_robot_ros_compat/ros_compat.h>

#include <aerial_robot_control/flight_navigation.h>
#if AERIAL_ROBOT_ROS_VERSION == 1
#  include <geometry_msgs/QuaternionStamped.h>
#  include <geometry_msgs/Vector3Stamped.h>
#  include <nav_msgs/Odometry.h>
#  include <sensor_msgs/JointState.h>
#  include <spinal/DesireCoord.h>
#else
#  include <geometry_msgs/msg/quaternion_stamped.hpp>
#  include <geometry_msgs/msg/vector3_stamped.hpp>
#  include <nav_msgs/msg/odometry.hpp>
#  include <sensor_msgs/msg/joint_state.hpp>
#  include <spinal/msg/desire_coord.hpp>
#endif
AERIAL_ROBOT_MSG_NAMESPACE(geometry_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(nav_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(sensor_msgs);
AERIAL_ROBOT_MSG_NAMESPACE(spinal);
#if AERIAL_ROBOT_ROS_VERSION == 1
#  include <kdl_conversions/kdl_msg.h>
#endif

namespace aerial_robot_navigation
{
  class DragonNavigator : public BaseNavigator
  {
  public:
    DragonNavigator();
    ~DragonNavigator(){}

    void initialize(ros_compat::NodeHandle nh, ros_compat::NodeHandle nhp,
                    ros_compat::SharedPtr<aerial_robot_model::RobotModel> robot_model,
                    ros_compat::SharedPtr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;

    inline const bool getEqCoGWorldFlag() const { return eq_cog_world_; }

  private:
    ros_compat::Publisher target_baselink_rpy_pub_; // to spinal
    ros_compat::Publisher joint_control_pub_;
    ros_compat::Subscriber final_target_baselink_rot_sub_, final_target_baselink_rpy_sub_;
    ros_compat::Subscriber target_rotation_motion_sub_;

    void halt() override;
    void reset() override;

    bool isInflightState() override;

    void servoTorqueProcess();
    void landingProcess();
    void gimbalControl();
    void baselinkRotationProcess();
    void rosParamInit() override;

    void targetBaselinkRotCallback(const ros_compat::ConstPtr<geometry_msgs_c::QuaternionStamped> & msg);
    void targetBaselinkRPYCallback(const ros_compat::ConstPtr<geometry_msgs_c::Vector3Stamped> & msg);
    void targetRotationMotionCallback(const ros_compat::ConstPtr<nav_msgs_c::Odometry>& msg);

    /* target baselink rotation */
    double prev_rotation_stamp_;
    std::vector<double> target_gimbal_angles_;
    tf2::Quaternion curr_target_baselink_rot_, final_target_baselink_rot_;
    bool eq_cog_world_;

    /* landing process */
    bool level_flag_;
    bool servo_torque_;
    double level_shape_control_stamp_;
    sensor_msgs_c::JointState level_shape_msg_;

    /* rosparam */
    double height_thresh_;
    string joints_torque_control_srv_name_, gimbals_torque_control_srv_name_;
    double baselink_rot_change_thresh_;
    double baselink_rot_pub_interval_;

    // addtional state 
    static constexpr uint8_t PRE_LAND_STATE = 0x20;
  };
};
