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

#include <aerial_robot_ros_compat/tf_compat.h>
#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <aerial_robot_estimation/sensor/imu.h>
#include <aerial_robot_estimation/state_estimation.h>

using namespace aerial_robot_estimation;

StateEstimator::StateEstimator()
  : sensor_fusion_flag_(false),
    qu_size_(0),
    flying_flag_(false),
    un_descend_flag_(false),
    force_att_control_flag_(false),
    has_groundtruth_odom_(false),
    imu_handlers_(0), alt_handlers_(0), vo_handlers_(0), gps_handlers_(0), plane_detection_handlers_(0)
{
  fuser_[0].resize(0);
  fuser_[1].resize(0);

  has_refined_yaw_estimate_[EGOMOTION_ESTIMATE] = false;
  has_refined_yaw_estimate_[EXPERIMENT_ESTIMATE] = false;

  for(int i = 0; i < 6; i ++)
    {
      for(int j = 0; j < 3; j++)
        {
          state_[i][j].first = 0;
          state_[i][j].second = tf2::Vector3(0, 0, 0);
        }
    }

  for(int i = 0; i < 3; i ++)
    {
      base_rots_.at(i).setIdentity();
      cog_rots_.at(i).setIdentity();
      base_omegas_.at(i).setZero();
      cog_omegas_.at(i).setZero();
      base_rot_status_.at(i) = 0;
      cog_rot_status_.at(i) = 0;
    }

  /* TODO: represented sensors unhealth level */
  unhealth_level_ = 0;
}

void StateEstimator::initialize(ros_compat::NodeHandle nh, ros_compat::NodeHandle nh_private, ros_compat::SharedPtr<aerial_robot_model::RobotModel> robot_model)
{
  nh_ = nh;
  nhp_ = nh_private;
  robot_model_ =robot_model;

  br_ = ros_compat::makeBroadcaster<tf2_ros::TransformBroadcaster>(nh_);

  rosParamInit();

  baselink_odom_pub_ = nh_.advertise<nav_msgs_c::Odometry>("uav/baselink/odom", 1);
  cog_odom_pub_ = nh_.advertise<nav_msgs_c::Odometry>("uav/cog/odom", 1);
  full_state_pub_ = nh_.advertise<aerial_robot_msgs_c::States>("uav/full_state", 1);

  nhp_.param("tf_prefix", tf_prefix_, std::string(""));

  double rate;
  nhp_.param("state_pub_rate", rate, 100.0);
  state_pub_timer_ = nh_.createTimer(ros_compat::duration(1.0 / rate), &StateEstimator::statePublish, this);
}


void StateEstimator::setOrientationWxB(int frame, int estimate_mode, tf2::Vector3 v)
{
  tf2::Vector3 wx_b = v.normalize();

  tf2::Matrix3x3 rot = getOrientation(frame, estimate_mode);
  tf2::Vector3 wz_b = rot.getRow(2);
  tf2::Vector3 wy_b = wz_b.cross(wx_b);
  wy_b.normalize();

  rot[0] = wx_b; rot[1] = wy_b; rot[2] = wz_b;

  setOrientation(frame, estimate_mode, rot);
}

void StateEstimator::setOrientationWzB(int frame, int estimate_mode, tf2::Vector3 v)
{
  tf2::Vector3 wz_b = v.normalize();

  tf2::Matrix3x3 rot = getOrientation(frame, estimate_mode);
  tf2::Vector3 wx_b = rot.getRow(0);
  tf2::Vector3 wy_b = wz_b.cross(wx_b);
  wy_b.normalize();

  rot[0] = wx_b; rot[1] = wy_b; rot[2] = wz_b;

  setOrientation(frame, estimate_mode, rot);
}

void StateEstimator::statePublish(const ros_compat::TimerEvent & e)
{
  static ros_compat::Time prev_pub_stamp = ros_compat::Time(0);

  ros_compat::Time imu_stamp = imu_handlers_.at(0)->getStamp();
  aerial_robot_msgs_c::States full_state;
  full_state.header.stamp = imu_stamp;

  for(int axis = 0; axis < 6; axis++)
    {
      aerial_robot_msgs_c::State r_state;
      AxisState state = getState(axis);

      switch(axis)
        {
        case State::X_COG:
          r_state.id = "x_cog";
          break;
        case State::Y_COG:
          r_state.id = "y_cog";
          break;
        case State::Z_COG:
          r_state.id = "z_cog";
          break;
        case State::X_BASE:
          r_state.id = "x_b";
          break;
        case State::Y_BASE:
          r_state.id = "y_b";
          break;
        case State::Z_BASE:
          r_state.id = "z_b";
          break;
        default:
          break;
        }
      r_state.state.resize(3);
      for(int mode = 0; mode < 3; mode++)
        ros_compat::vector3TfToMsg(state[mode].second, r_state.state[mode]);

      full_state.states.push_back(r_state);
    }
  full_state_pub_.publish(full_state);

  nav_msgs_c::Odometry odom_state;
  odom_state.header.stamp = imu_stamp;
  odom_state.header.frame_id = std::string("/world");

  /* Baselink */
  /* Rotation */
  tf2::Quaternion q; getOrientation(Frame::BASELINK, estimate_mode_).getRotation(q);
  q.normalize();
  ros_compat::quaternionTfToMsg(q, odom_state.pose.pose.orientation);
  ros_compat::vector3TfToMsg(getAngularVel(Frame::BASELINK, estimate_mode_), odom_state.twist.twist.angular);

  /* Translation */
  odom_state.child_frame_id = ros_compat::resolveFrame(tf_prefix_, robot_model_->getBaselinkName());
  ros_compat::pointTfToMsg(getPos(Frame::BASELINK, estimate_mode_), odom_state.pose.pose.position);
  ros_compat::vector3TfToMsg(getVel(Frame::BASELINK, estimate_mode_), odom_state.twist.twist.linear);
  baselink_odom_pub_.publish(odom_state);

  /* TF broadcast from world frame */
  /* avoid the redundant timestamp which induces annoying loggings from TF server */
  if (imu_stamp != prev_pub_stamp)
    {
      tf2::Transform root2baselink_tf;
      const auto segments_tf = robot_model_->getSegmentsTf();
      if(segments_tf.size() > 0) // kinemtiacs is initialized
        ros_compat::transformKdlToTf(segments_tf.at(robot_model_->getBaselinkName()), root2baselink_tf);
      else
        root2baselink_tf.setIdentity(); // not initialized

      tf2::Transform world2baselink_tf;
      ros_compat::poseMsgToTf(odom_state.pose.pose, world2baselink_tf);
      // Built field by field rather than through tf::StampedTransform, which
      // ROS2 has no equivalent of - tf2 carries the stamp and frame names on
      // the message itself rather than on the transform type.
      geometry_msgs_c::TransformStamped transformStamped;
      ros_compat::transformTfToMsg(world2baselink_tf * root2baselink_tf.inverse(), transformStamped.transform);
      transformStamped.header.stamp = imu_stamp;
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = ros_compat::resolveFrame(tf_prefix_, std::string("root"));
      br_->sendTransform(transformStamped);
    }

  /* COG */
  /* Rotation */
  getOrientation(Frame::COG, estimate_mode_).getRotation(q);
  q.normalize();
  ros_compat::quaternionTfToMsg(q, odom_state.pose.pose.orientation);
  ros_compat::vector3TfToMsg(getAngularVel(Frame::COG, estimate_mode_), odom_state.twist.twist.angular);
  /* Translation */
  odom_state.child_frame_id = ros_compat::resolveFrame(tf_prefix_, std::string("cog"));
  ros_compat::pointTfToMsg(getPos(Frame::COG, estimate_mode_), odom_state.pose.pose.position);
  ros_compat::vector3TfToMsg(getVel(Frame::COG, estimate_mode_), odom_state.twist.twist.linear);
  cog_odom_pub_.publish(odom_state);

  prev_pub_stamp = imu_stamp;
}


void StateEstimator::rosParamInit()
{
  auto pattern_match = [](std::string &pl, std::string &pl_candidate)
  {
    int cmp = fnmatch(pl.c_str(), pl_candidate.c_str(), FNM_CASEFOLD);
    if (cmp == 0)
      return true;
    else if (cmp != FNM_NOMATCH) {
      // never see that, i think that it is fatal error.
      ROS_COMPAT_FATAL("Plugin list check error! fnmatch('%s', '%s', FNM_CASEFOLD) -> %d",
                pl.c_str(), pl_candidate.c_str(), cmp);
      ros_compat::shutdown();
    }
    return false;
  };

  nhp_.param ("param_verbose", param_verbose_, true);

  ros_compat::NodeHandle nh = ros_compat::NodeHandle(nh_, "estimation");
  nh.param ("mode", estimate_mode_, 0); //EGOMOTION_ESTIMATE: 0
  ROS_COMPAT_WARN("mode is %s", (estimate_mode_ == EGOMOTION_ESTIMATE)?string("EGOMOTION_ESTIMATE").c_str():((estimate_mode_ == EXPERIMENT_ESTIMATE)?string("EXPERIMENT_ESTIMATE").c_str():((estimate_mode_ == GROUND_TRUTH)?string("GROUND_TRUTH").c_str():string("WRONG_MODE").c_str())));

  sensor_fusion_loader_ptr_ = ros_compat::SharedPtr< pluginlib::ClassLoader<kf_plugin::KalmanFilter> >(new pluginlib::ClassLoader<kf_plugin::KalmanFilter>("kalman_filter", "kf_plugin::KalmanFilter"));

  /* kalman filter egomotion plugin initialization for 0: egomotion, 1: experiment */
  for (int i = 0; i < 2; i++)
    {
      /* kalman filter egomotion plugin list */
      std::vector<std::string> pl_list{};
      string prefix;
      if(i == EGOMOTION_ESTIMATE) prefix = string("egomotion");
      else if(i == EXPERIMENT_ESTIMATE) prefix = string("experiment");

      nh.getParam(prefix + "_list", pl_list);

      for (auto &pl_name : pl_list)
        {
          for (auto &name : sensor_fusion_loader_ptr_->getDeclaredClasses())
            {
              if(!pattern_match(pl_name, name)) continue;

              std::stringstream fuser_no;
              fuser_no << fuser_[i].size() + 1;

              int fuser_id;
              string fuser_name;

              if (!nh.getParam ("fuser_" + prefix + "_id" + fuser_no.str(), fuser_id))
                ROS_COMPAT_ERROR("%s, no param in fuser %s id", prefix.c_str(), fuser_no.str().c_str());

              if (!nh.getParam ("fuser_" + prefix + "_name" + fuser_no.str(), fuser_name))
                ROS_COMPAT_ERROR("%s, no param in fuser %s name", prefix.c_str(), fuser_no.str().c_str());

              ros_compat::SharedPtr<kf_plugin::KalmanFilter> plugin_ptr = ros_compat::createPluginInstance(*sensor_fusion_loader_ptr_, name);
              plugin_ptr->initialize(fuser_name, fuser_id);
              fuser_[i].push_back(make_pair(name, plugin_ptr));
              break;
            }
        }
    }

  sensor_plugin_ptr_ =  ros_compat::SharedPtr< pluginlib::ClassLoader<sensor_plugin::SensorBase> >( new pluginlib::ClassLoader<sensor_plugin::SensorBase>("aerial_robot_estimation", "sensor_plugin::SensorBase"));

  std::vector<std::string> sensor_list{};
  nh.getParam("sensor_list", sensor_list);

  vector<int> sensor_index(0);

  for (auto &sensor_plugin_name : sensor_list)
    {
      for (auto &name : sensor_plugin_ptr_->getDeclaredClasses())
        {
          if(!pattern_match(sensor_plugin_name, name)) continue;

          sensors_.push_back(ros_compat::createPluginInstance(*sensor_plugin_ptr_, name));
          sensor_index.push_back(1);

          if(name.find("imu") != std::string::npos)
            {
              imu_handlers_.push_back(sensors_.back());
              sensor_index.back() = imu_handlers_.size();
            }

          if(name.find("gps") != std::string::npos)
            {
              gps_handlers_.push_back(sensors_.back());
              sensor_index.back() = gps_handlers_.size();
            }

          if(name.find("alt") != std::string::npos)
            {
              alt_handlers_.push_back(sensors_.back());
              sensor_index.back() = alt_handlers_.size();
            }

          if(name.find("vo") != std::string::npos)
            {
              vo_handlers_.push_back(sensors_.back());
              sensor_index.back() = vo_handlers_.size();
            }

          if(name.find("plane_detection") != std::string::npos)
            {
              plane_detection_handlers_.push_back(sensors_.back());
              sensor_index.back() = plane_detection_handlers_.size();
            }


          sensors_.back()->initialize(nh_, robot_model_, shared_from_this(), name, sensor_index.back());
          break;
        }
    }
}
