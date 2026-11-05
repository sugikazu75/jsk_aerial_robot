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


#include <aerial_robot_model/servo_bridge.h>
#include <aerial_robot_model/servo_backend/dynamixel_backend.h>
#include <aerial_robot_model/servo_backend/mit_can/mit_can_backend.h>

ServoBridge::ServoBridge(ros::NodeHandle nh, ros::NodeHandle nhp): nh_(nh),nhp_(nhp), real_state_received_(false)
{
  nh_.param("/use_sim_time", simulation_mode_, false);
  nhp_.param("joint_state_pub_rate", joint_state_pub_rate_, 100.0); // [Hz]
  nhp_.param("state_timeout", state_timeout_, 1.0); // [sec], warn if the feedback from the real servo stops
  nhp_.param("use_servo_state_stamp", use_servo_state_stamp_, false); // true: latest stamp of servo states, false: ros::Time::now()
  double mit_command_rate;
  nhp_.param("mit_command_rate", mit_command_rate, 500.0); // [Hz], re-send the latest command to the motors with MIT mode (e.g., cubemars)
  if(simulation_mode_)
    {
      if(!ros::service::waitForService(nh_.getNamespace() + std::string("/controller_manager/load_controller")))
        {
          ROS_ERROR("can't find the controller manager: %s", (nh_.getNamespace() + std::string("/controller_manager/load_controller")).c_str());
          return;
        }
    }

  /* get robot URDF model via robot_description*/
  urdf::Model urdf_model;
  if (!urdf_model.initParam("robot_description"))
    ROS_ERROR("Failed to extract urdf model from rosparam");

  /* common param and topics (between servo_bridge and spinal_ros_bridge) */
  ros::NodeHandle nh_controller(nh_, "servo_controller");
  std::string state_sub_topic, pos_pub_topic, torque_pub_topic, servo_enable_pub_topic;
  nh_controller.param("state_sub_topic", state_sub_topic, std::string("servo/states"));
  nh_controller.param("pos_pub_topic", pos_pub_topic, std::string("servo/target_states"));
  nh_controller.param("torque_pub_topic", torque_pub_topic, std::string("servo/target_current"));
  nh_controller.param("servo_enable_pub_topic", servo_enable_pub_topic, std::string("servo/torque_enable"));
  /* common subsriber: get servo states (i.e. joint angles) from real machine (spinal_ros_bridge) */
  servo_states_subs_.insert(make_pair("common", nh_.subscribe<spinal::ServoStates>(state_sub_topic, 10, boost::bind(&ServoBridge::servoStatesCallback, this, _1, "common"))));
  /* common publisher: target servo position to real machine (spinal_ros_bridge) */
  servo_target_pos_pubs_.insert(make_pair("common", nh_.advertise<spinal::ServoControlCmd>(pos_pub_topic, 1)));
  /* common publisher: target servo torque to real machine (spinal_ros_bridge) */
  servo_target_torque_pubs_.insert(make_pair("common", nh_.advertise<spinal::ServoControlCmd>(torque_pub_topic, 1)));
  /* common publisher: servo on/off flag to real machine (spinal_ros_bridge) */
  servo_enable_pubs_.insert(make_pair("common", nh_.advertise<spinal::ServoTorqueCmd>(servo_enable_pub_topic, 1)));
  /* common publisher: joint profiles */
  joint_profile_pub_ = nh_.advertise<spinal::JointProfiles>("joint_profiles",1);
  /* subscriber: uav info */
  uav_info_sub_ = nh_.subscribe<spinal::UavInfo>("uav_info", 1, &ServoBridge::uavInfoCallback, this);

  /* backend to send commands to real machine: dynamixel via spinal (one usb serial) */
  servo_backends_.insert(make_pair(ServoType::DYNAMIXEL, std::make_shared<DynamixelBackend>(servo_target_pos_pubs_.at("common"), servo_target_torque_pubs_.at("common"), servo_enable_pubs_.at("common"))));
  /* backend for the motors with MIT mode via SocketCAN, CAN is not used in simulation */
  ServoBackendPtr mit_can_backend = std::make_shared<MitCanBackend>(mutex_, !simulation_mode_, mit_command_rate);
  servo_backends_.insert(make_pair(ServoType::CUBEMARS, mit_can_backend));
  servo_backends_.insert(make_pair(ServoType::XIAOMI, mit_can_backend));


  /* get additional config for servos from ros parameters */
  XmlRpc::XmlRpcValue all_servos_params;
  nh_.getParam("servo_controller", all_servos_params);
  spinal::JointProfiles joint_profiles_msg;
  for(auto servo_group_params: all_servos_params)
    {
      if (servo_group_params.second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
        continue;

      std::string group_name = servo_group_params.first;

      /* check whether has feedback state from real machine */
      no_real_state_flags_.insert(make_pair(group_name, false));
      if(servo_group_params.second.hasMember("no_real_state"))
        no_real_state_flags_.at(group_name) = servo_group_params.second["no_real_state"];

      /* servo type (default: dynamixel) */
      std::string group_servo_type = ServoType::DYNAMIXEL;
      if(servo_group_params.second.hasMember("type"))
        group_servo_type = (string)servo_group_params.second["type"];

      /* pub, sub, and srv */
      /* mandatory subscriber: target servo command (e.g., position, torque) from controller */
      servo_ctrl_subs_.insert(make_pair(group_name, nh_.subscribe<sensor_msgs::JointState>(group_name + string("_ctrl"), 10, boost::bind(&ServoBridge::servoCtrlCallback, this, _1, group_name))));
      /* mandatory subscriber: only target servo troque command from controller */
      servo_torque_ctrl_subs_.insert(make_pair(group_name, nh_.subscribe<sensor_msgs::JointState>(group_name + string("_torque_ctrl"), 10, boost::bind(&ServoBridge::servoTorqueCtrlCallback, this, _1, group_name))));
      /* mandatory service: get torque enalbe/disable flag  */
      servo_enable_srvs_.insert(make_pair(group_name, nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(servo_group_params.first + string("/torque_enable"), boost::bind(&ServoBridge::servoEnableCallback, this, _1, _2, servo_group_params.first))));
      /* service: correct the multi-turn ambiguity of the joint angle (e.g., cubemars motor with single encoder) */
      servo_rough_calib_srvs_.insert(make_pair(group_name, nh_.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(servo_group_params.first + string("/rough_calib"), boost::bind(&ServoBridge::servoRoughCalibCallback, this, _1, _2, servo_group_params.first))));

      if(servo_group_params.second.hasMember("state_sub_topic"))
        {
          /* option: servo states (i.e. joint angles) from real machine (spinal_ros_bridge) */
          servo_states_subs_.insert(make_pair(group_name, nh_.subscribe<spinal::ServoStates>((string)servo_group_params.second["state_sub_topic"], 10, boost::bind(&ServoBridge::servoStatesCallback, this, _1, servo_group_params.first))));
        }

      if(servo_group_params.second.hasMember("pos_pub_topic") ||
         servo_group_params.second.hasMember("torque_pub_topic") ||
         servo_group_params.second.hasMember("servo_enable_pub_topic"))
        {
          /* commands are sent via the servo backend with the common topics */
          ROS_WARN("[servo bridge]: group specific pos_pub_topic, torque_pub_topic and servo_enable_pub_topic for %s are not supported, use the common topics", group_name.c_str());
        }

      /* servo handler */
      ServoGroupHandler servo_group_handler;
      for(auto servo_params : servo_group_params.second)
        {
          if(servo_params.first.find("controller") != string::npos)
            {
              /* servo type: servo specific param > group param > dynamixel */
              std::string servo_type = servo_params.second.hasMember("type")?
                (string)servo_params.second["type"]: group_servo_type;
              if(servo_backends_.find(servo_type) == servo_backends_.end())
                {
                  ROS_ERROR("[servo bridge]: unsupported servo type '%s' for %s, skip", servo_type.c_str(), string(servo_params.second["name"]).c_str());
                  continue;
                }

              /* get parameters from urdf file */
              double upper_limit = urdf_model.getJoint(servo_params.second["name"])->limits->upper;
              double lower_limit = urdf_model.getJoint(servo_params.second["name"])->limits->lower;

              /* get parameters from rosparam */
              int servo_id = servo_params.second["id"]; 
              int angle_sgn = 1;
              int zero_point_offset = 0; // only for dynamixel (servo unit)
              double angle_scale = 1.0; // only for dynamixel (servo unit)
              if(servo_type == ServoType::DYNAMIXEL)
                {
                  angle_sgn = servo_params.second.hasMember("angle_sgn")? 
                    servo_params.second["angle_sgn"]:servo_group_params.second["angle_sgn"];
                  zero_point_offset = servo_params.second.hasMember("zero_point_offset")?
                    servo_params.second["zero_point_offset"]:servo_group_params.second["zero_point_offset"];
                  angle_scale = servo_params.second.hasMember("angle_scale")?
                    servo_params.second["angle_scale"]:servo_group_params.second["angle_scale"];
                }
              else
                {
                  /* other servos handle the unit conversion (e.g., zero point offset in radian) in the backend */
                  if(servo_params.second.hasMember("angle_sgn")) angle_sgn = servo_params.second["angle_sgn"];
                  else if(servo_group_params.second.hasMember("angle_sgn")) angle_sgn = servo_group_params.second["angle_sgn"];
                }

              double torque_scale = servo_group_params.second.hasMember("torque_scale")?
                servo_group_params.second["torque_scale"]:(servo_params.second.hasMember("torque_scale")?servo_params.second["torque_scale"]: XmlRpc::XmlRpcValue(1.0));

              /* for low pass filtering */
              bool filter_flag = servo_group_params.second.hasMember("filter_flag")?
                servo_group_params.second["filter_flag"]:(servo_params.second.hasMember("filter_flag")?servo_params.second["filter_flag"]: XmlRpc::XmlRpcValue(false));
              double sample_freq = servo_group_params.second.hasMember("sample_freq")?
                servo_group_params.second["sample_freq"]:(servo_params.second.hasMember("sample_freq")?servo_params.second["sample_freq"]: XmlRpc::XmlRpcValue(0.0));
              double cutoff_freq = servo_group_params.second.hasMember("cutoff_freq")?
                servo_group_params.second["cutoff_freq"]:(servo_params.second.hasMember("cutoff_freq")?servo_params.second["cutoff_freq"]: XmlRpc::XmlRpcValue(0.0));

              servo_group_handler.push_back(SingleServoHandlePtr(new SingleServoHandle(servo_params.second["name"], servo_params.second["id"], angle_sgn, zero_point_offset, angle_scale, upper_limit, lower_limit, torque_scale, !no_real_state_flags_.at(group_name), filter_flag, sample_freq, cutoff_freq)));
              servo_group_handler.back()->setType(servo_type);
              servo_group_handler.back()->setBackend(servo_backends_.at(servo_type));
              if(!servo_backends_.at(servo_type)->addServo(servo_group_handler.back().get(), servo_group_params.second, servo_params.second))
                {
                  ROS_ERROR("[servo bridge]: failed to add %s to %s backend, skip", string(servo_params.second["name"]).c_str(), servo_type.c_str());
                  servo_group_handler.pop_back();
                  continue;
                }

              /* rosparam and load controller for gazebo */
              if(simulation_mode_)
                {
                  if(!servo_group_params.second.hasMember("simulation") &&
                     !servo_params.second.hasMember("simulation"))
                    {
                      ROS_ERROR("please set gazebo servo parameters for %s, using sub namespace 'simulation:'", string(servo_params.second["name"]).c_str());

                      continue;
                    }

                  vector<string> parameter_names = {"type", "pid", "init_value"};
                  for(auto parameter_name : parameter_names)
                    {
                      if(!servo_params.second.hasMember("simulation") ||
                         (servo_params.second.hasMember("simulation") &&
                          !servo_params.second["simulation"].hasMember(parameter_name)))
                        {
                          if(!servo_group_params.second["simulation"].hasMember(parameter_name))
                            {
                              ROS_ERROR("can not find '%s' gazebo paramter for servo %s", parameter_name.c_str(),  string(servo_params.second["name"]).c_str());
                              return;
                            }
                          nh_.setParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/") + parameter_name, servo_group_params.second["simulation"][parameter_name]);
                        }
                    }
                  nh_.setParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/joint"), servo_params.second["name"]);


                  /* load the controller */
                  ros::ServiceClient controller_loader = nh_.serviceClient<controller_manager_msgs::LoadController>(nh_.getNamespace() + std::string("/controller_manager/load_controller"));
                  controller_manager_msgs::LoadController load_srv;
                  load_srv.request.name = nh_.getNamespace() + string("/servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation");

                  if (controller_loader.call(load_srv))
                    ROS_INFO("load gazebo controller for %s", string(servo_params.second["name"]).c_str());
                  else
                    ROS_ERROR("Failed to call service %s", controller_loader.getService().c_str());

                  /* start the controller */
                  ros::ServiceClient controller_starter = nh_.serviceClient<controller_manager_msgs::SwitchController>(nh_.getNamespace() + std::string("/controller_manager/switch_controller"));
                  controller_manager_msgs::SwitchController switch_srv;
                  switch_srv.request.start_controllers.push_back(load_srv.request.name);
                  switch_srv.request.strictness = controller_manager_msgs::SwitchController::Request::STRICT;

                  if (controller_starter.call(switch_srv))
                    ROS_INFO("start servo controller for %s", string(servo_params.second["name"]).c_str());
                  else
                    ROS_ERROR("Failed to call service %s", controller_starter.getService().c_str());

                  /* init the servo command publisher to the controller */
                  servo_target_pos_sim_pubs_[servo_group_params.first].push_back(nh_.advertise<std_msgs::Float64>(load_srv.request.name + string("/command"), 1));
                  servo_target_commands_sim_pubs_[servo_group_params.first].push_back(nh_.advertise<std_msgs::Float64MultiArray>(load_srv.request.name + string("/commands"), 1));
                  // wait for the publisher initialization
                  while(servo_target_pos_sim_pubs_[servo_group_params.first].back().getNumSubscribers() == 0 && ros::ok())
                    ros::Duration(0.1).sleep();

                  /* set the initial angle */
                  std_msgs::Float64 msg;
                  nh_.getParam(string("servo_controller/") + servo_group_params.first + string("/") + servo_params.first + string("/simulation/init_value"), msg.data);
                  servo_target_pos_sim_pubs_[servo_group_params.first].back().publish(msg);
                }
            }
        }

      servos_handler_.insert(make_pair(servo_group_params.first, servo_group_handler));
    }

  if(!simulation_mode_)
    {
      /* if no servo has feedback from real machine, publish joint states from the beginning */
      real_state_received_ = true;
      for(const auto& servo_group : servos_handler_)
        for(const auto& servo_handler : servo_group.second)
          if(servo_handler->receiveRealState()) real_state_received_ = false;

      servo_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
      joint_state_timer_ = nh_.createTimer(ros::Duration(1.0 / joint_state_pub_rate_), &ServoBridge::jointStatePublish, this);
    }
}

void ServoBridge::servoStatesCallback(const spinal::ServoStatesConstPtr& state_msg, const string& servo_group_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const ros::Time now = ros::Time::now();
  latest_state_stamp_ = state_msg->stamp;
  real_state_received_ = true;

  /* servo states from spinal are only for dynamixel */
  /* joint states are published periodically in jointStatePublish() */

  /* independent group servo process */
  if(servo_group_name != std::string("common"))
    {
      for(auto it: state_msg->servos)
        {
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(),
                                       servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return s->getType() == ServoType::DYNAMIXEL && it.index == s->getId();} );

          if(servo_handler == servos_handler_[servo_group_name].end())
            {
              ROS_ERROR("[servo bridge, servo state callback]: no matching joint handler for servo index %d", it.index);
              return;
            }
          (*servo_handler)->setCurrAngleVal((double)it.angle, ValueType::BIT); // angle (position)
          (*servo_handler)->setCurrTorqueVal((double)it.load); // torque (effort)
          (*servo_handler)->markStateReceived(now);
        }

      // finish process
      return;
    }


  /* group of "common", search in all group */
  for(auto it: state_msg->servos)
    {
      for(auto servo_group: servos_handler_)
        {
          // no real state from real machine, skip
          if(no_real_state_flags_.at(servo_group.first))
            {
              ROS_DEBUG_STREAM(servo_group.first << ", no real state, skip");
              continue;
            }

          // has independent servo state process
          if(servo_states_subs_.find(servo_group.first) != servo_states_subs_.end())
            {
              ROS_DEBUG_STREAM(servo_group.first << ", do not process in common group, skip");
              continue;
            }

          auto servo_handler = find_if(servos_handler_[servo_group.first].begin(),
                                       servos_handler_[servo_group.first].end(),
                                       [&](SingleServoHandlePtr s) {return s->getType() == ServoType::DYNAMIXEL && it.index == s->getId();});

          if(servo_handler == servos_handler_[servo_group.first].end())
            {
              // search in next servo group
              continue;
            }

          (*servo_handler)->setCurrAngleVal((double)it.angle, ValueType::BIT); // angle (position)
          (*servo_handler)->setCurrTorqueVal((double)it.load); // torque (effort)
          (*servo_handler)->markStateReceived(now);

          ROS_DEBUG("servo index: %d, find in group %s", it.index, servo_group.first.c_str());

          break; // search next servo state from real machine
        }
    }
}

void ServoBridge::jointStatePublish(const ros::TimerEvent& e)
{
  sensor_msgs::JointState servo_states_msg;
  std::string stale_servos;

  {
    std::lock_guard<std::mutex> lock(mutex_);

    // wait for the first feedback from real machine to avoid publishing the dummy initial value
    if(!real_state_received_)
      {
        // the servo states can be also updated in the backend (e.g., CAN receive thread)
        for(const auto& servo_group : servos_handler_)
          for(const auto& servo_handler: servo_group.second)
            if(servo_handler->stateReceived()) real_state_received_ = true;
      }
    if(!real_state_received_) return;

    const ros::Time now = ros::Time::now();
    servo_states_msg.header.stamp = use_servo_state_stamp_ ? latest_state_stamp_ : now;

    // publish velocity only if some servos provide velocity (0 for the other servos)
    bool velocity_available = false;
    for(const auto& servo_group : servos_handler_)
      for(const auto& servo_handler: servo_group.second)
        if(servo_handler->velocityAvailable()) velocity_available = true;

    for(const auto& servo_group : servos_handler_)
      {
        for(const auto& servo_handler: servo_group.second)
          {
            servo_states_msg.name.push_back(servo_handler->getName());
            servo_states_msg.position.push_back(servo_handler->getCurrAngleVal(ValueType::RADIAN));
            servo_states_msg.effort.push_back(servo_handler->getCurrTorqueVal());
            if(velocity_available) servo_states_msg.velocity.push_back(servo_handler->getCurrVelocityVal());

            if(servo_handler->receiveRealState() &&
               (!servo_handler->stateReceived() ||
                (now - servo_handler->getLastStateTime()).toSec() > state_timeout_))
              stale_servos += servo_handler->getName() + std::string(" ");
          }
      }
  }

  servo_states_pub_.publish(servo_states_msg);

  if(!stale_servos.empty())
    ROS_WARN_THROTTLE(5.0, "[servo bridge]: no state update from real servo for more than %.2f [sec]: %s", state_timeout_, stale_servos.c_str());
}

void ServoBridge::flushBackends()
{
  for(auto& backend : servo_backends_) backend.second->flush();
}

void ServoBridge::clearBackends()
{
  for(auto& backend : servo_backends_) backend.second->clear();
}

void ServoBridge::publishSimCommand(const std::string& servo_group_name, int index, double position, bool has_vel_torque, double velocity, double ff_torque)
{
  auto pos_pubs = servo_target_pos_sim_pubs_.find(servo_group_name);
  if(pos_pubs == servo_target_pos_sim_pubs_.end() || index >= (int)pos_pubs->second.size())
    {
      ROS_ERROR_THROTTLE(1.0, "[servo bridge]: no simulation controller for servo %d in group %s", index, servo_group_name.c_str());
      return;
    }

  /*
    Send to only one of "command" and "commands".
    "command" (position only) resets the velocity and feed-forward torque in the controller (e.g., JointComputeTorqueController),
    so sending both topics can overwrite the velocity and ff torque with zero.
  */
  const ros::Publisher& commands_pub = servo_target_commands_sim_pubs_.at(servo_group_name).at(index);
  if(has_vel_torque && commands_pub.getNumSubscribers() > 0)
    {
      // pos, vel, ff_torque
      std_msgs::Float64MultiArray pos_vel_torque_msg;
      pos_vel_torque_msg.data.push_back(position);
      pos_vel_torque_msg.data.push_back(velocity);
      pos_vel_torque_msg.data.push_back(ff_torque);
      commands_pub.publish(pos_vel_torque_msg);
      return;
    }

  std_msgs::Float64 msg;
  msg.data = position;
  pos_pubs->second.at(index).publish(msg);
}

void ServoBridge::servoCtrlCallback(const sensor_msgs::JointStateConstPtr& servo_ctrl_msg, const string& servo_group_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if(servo_ctrl_msg->name.size() > 0)
    {
      for(int i = 0; i < servo_ctrl_msg->name.size(); i++)
        {/* servo name is assigned */
          if(servo_ctrl_msg->position.size() !=  servo_ctrl_msg->name.size())
            {
              ROS_ERROR("[servo bridge, servo control control]: the servo position num and name num are different in ros msgs [%d vs %d]",
                        (int)servo_ctrl_msg->position.size(), (int)servo_ctrl_msg->name.size());
              return;
            }

          // use servo_name to search the servo_handler
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(), servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return servo_ctrl_msg->name.at(i)  == s->getName();});

          if(servo_handler == servos_handler_[servo_group_name].end())
          {
            ROS_ERROR("[servo bridge, servo control callback]: no matching servo handler for %s", servo_ctrl_msg->name.at(i).c_str());
            clearBackends();
            return;
          }

          ServoCommand cmd;
          cmd.has_position = true;
          cmd.position = servo_ctrl_msg->position[i];
          (*servo_handler)->setTargetAngleVal(cmd.position, ValueType::RADIAN);

          if(servo_ctrl_msg->velocity.size() == servo_ctrl_msg->name.size())
            {
              cmd.has_velocity = true;
              cmd.velocity = servo_ctrl_msg->velocity.at(i);
            }

          // process torque command if necessary
          if(servo_ctrl_msg->effort.size() == servo_ctrl_msg->name.size())
            {
              cmd.has_effort = true;
              cmd.effort = servo_ctrl_msg->effort.at(i);
              (*servo_handler)->setTargetTorqueVal(cmd.effort);
            }

          (*servo_handler)->getBackend()->addCommand(**servo_handler, cmd);

          if(simulation_mode_)
            {
              // pos, vel, ff_torque
              bool has_vel_torque = (servo_ctrl_msg->position.size() == servo_ctrl_msg->velocity.size()) && (servo_ctrl_msg->position.size() == servo_ctrl_msg->effort.size());
              publishSimCommand(servo_group_name, distance(servos_handler_[servo_group_name].begin(), servo_handler),
                                servo_ctrl_msg->position.at(i), has_vel_torque,
                                has_vel_torque ? servo_ctrl_msg->velocity.at(i): 0.0,
                                has_vel_torque ? servo_ctrl_msg->effort.at(i): 0.0);
            }
        }
    }
  else
    { /* for fast tranmission: no searching process, in the predefine order */

      if(servo_ctrl_msg->position.size() != servos_handler_[servo_group_name].size())
        {
          ROS_ERROR("[servo bridge, servo ctrl control]: the joint num from rosparam %d is not equal with ros msgs %d",
                    (int)servos_handler_[servo_group_name].size(), (int)servo_ctrl_msg->position.size());
          return;
        }

      for(int i = 0; i < servo_ctrl_msg->position.size(); i++)
        {
          /*  use the kinematics order (e.g. joint1 ~ joint N, gimbal_roll -> gimbal_pitch) */
          SingleServoHandlePtr servo_handler = servos_handler_[servo_group_name].at(i);
          ServoCommand cmd;
          cmd.has_position = true;
          cmd.position = servo_ctrl_msg->position[i];
          servo_handler->setTargetAngleVal(cmd.position, ValueType::RADIAN);

          if(servo_ctrl_msg->velocity.size() == servo_ctrl_msg->position.size())
            {
              cmd.has_velocity = true;
              cmd.velocity = servo_ctrl_msg->velocity.at(i);
            }

          // process torque command if necessary
          if(servo_ctrl_msg->effort.size() == servo_ctrl_msg->position.size())
            {
              cmd.has_effort = true;
              cmd.effort = servo_ctrl_msg->effort.at(i);
              servo_handler->setTargetTorqueVal(cmd.effort);
            }

          servo_handler->getBackend()->addCommand(*servo_handler, cmd);

          if(simulation_mode_)
            {
              // pos, vel, ff_torque
              bool has_vel_torque = (servo_ctrl_msg->position.size() == servo_ctrl_msg->velocity.size()) && (servo_ctrl_msg->position.size() == servo_ctrl_msg->effort.size());
              publishSimCommand(servo_group_name, i,
                                servo_ctrl_msg->position.at(i), has_vel_torque,
                                has_vel_torque ? servo_ctrl_msg->velocity.at(i): 0.0,
                                has_vel_torque ? servo_ctrl_msg->effort.at(i): 0.0);
            }
        }
    }

  flushBackends();
}

void ServoBridge::servoTorqueCtrlCallback(const sensor_msgs::JointStateConstPtr& servo_ctrl_msg, const string& servo_group_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if(servo_ctrl_msg->name.size() > 0)
    {
      for(int i = 0; i < servo_ctrl_msg->name.size(); i++)
        {/* servo name is assigned */

          if(servo_ctrl_msg->effort.size() !=  servo_ctrl_msg->name.size())
            {
              ROS_ERROR("[servo bridge, servo torque control]: the servo effort (torque) num and name num are different in ros msgs [%d vs %d]",
                        (int)servo_ctrl_msg->effort.size(), (int)servo_ctrl_msg->name.size());
              return;
            }

          // use servo_name to search the servo_handler
          auto servo_handler = find_if(servos_handler_[servo_group_name].begin(), servos_handler_[servo_group_name].end(),
                                       [&](SingleServoHandlePtr s) {return servo_ctrl_msg->name.at(i)  == s->getName();});

          if(servo_handler == servos_handler_[servo_group_name].end())
          {
            ROS_ERROR("[servo bridge, servo control callback]: no matching servo handler for %s", servo_ctrl_msg->name.at(i).c_str());
            clearBackends();
            return;
          }

          ServoCommand cmd;
          cmd.has_effort = true;
          cmd.effort = servo_ctrl_msg->effort.at(i);
          (*servo_handler)->setTargetTorqueVal(cmd.effort);
          (*servo_handler)->getBackend()->addCommand(**servo_handler, cmd);
        }
    }
  else
    { /* for fast tranmission: no searching process, in the predefine order */

      if(servo_ctrl_msg->effort.size() != servos_handler_[servo_group_name].size())
        {
          ROS_ERROR("[servo bridge, servo torque control]: the joint num from rosparam %d is not equal with ros msgs %d",
                    (int)servos_handler_[servo_group_name].size(), (int)servo_ctrl_msg->effort.size());
          return;
        }

      for(int i = 0; i < servo_ctrl_msg->effort.size(); i++)
        {
          /*  use the kinematics order (e.g. joint1 ~ joint N, gimbal_roll -> gimbal_pitch) */
          SingleServoHandlePtr servo_handler = servos_handler_[servo_group_name].at(i);
          ServoCommand cmd;
          cmd.has_effort = true;
          cmd.effort = servo_ctrl_msg->effort.at(i);
          servo_handler->setTargetTorqueVal(cmd.effort);
          servo_handler->getBackend()->addCommand(*servo_handler, cmd);
        }
    }

  flushBackends();
}

bool ServoBridge::servoEnableCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& servo_group_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  res.success = true;
  for(auto servo_handler: servos_handler_[servo_group_name])
    {
      if(!servo_handler->getBackend()->addTorqueEnable(*servo_handler, req.data))
        {
          res.success = false;
          res.message += servo_handler->getName() + string(" ");
        }
    }

  flushBackends();

  if(!res.success) res.message = string("failed to set torque: ") + res.message;
  return true;
}

bool ServoBridge::servoRoughCalibCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res, const std::string& servo_group_name)
{
  std::lock_guard<std::mutex> lock(mutex_);

  res.success = true;
  for(auto servo_handler: servos_handler_[servo_group_name])
    {
      if(servo_handler->getType() == ServoType::DYNAMIXEL) continue; // absolute encoder

      std::string message;
      if(!servo_handler->getBackend()->roughCalib(*servo_handler, message)) res.success = false;
      res.message += message + string("; ");
    }

  return true;
}

void ServoBridge::uavInfoCallback(const spinal::UavInfoConstPtr& uav_msg)
{
  /* Send servo profiles to Spinal*/
  spinal::JointProfiles joint_profiles_msg;
  for(auto servo_group : servos_handler_){
    for(auto servo : servo_group.second){
      /* only dynamixel servos are handled by spinal */
      if(servo->getType() != ServoType::DYNAMIXEL) continue;

      spinal::JointProfile joint_profile;
      if(servo_group.first == "joints"){
        joint_profile.type = spinal::JointProfile::JOINT;
      }
      else if(servo_group.first == "gimbals"){
        joint_profile.type = spinal::JointProfile::GIMBAL;
      }
      else{
        ROS_ERROR("Invalid servo type. Please define 'joints' or 'gimbals'.");
        continue;
      }
      joint_profile.servo_id = servo->getId();
      joint_profile.angle_sgn = servo->getAngleSgn();
      joint_profile.angle_scale = servo->getAngleScale();
      joint_profile.zero_point_offset = servo->getZeroPointOffset();
      joint_profiles_msg.joints.push_back(joint_profile);
    }
  }
  joint_profile_pub_.publish(joint_profiles_msg);
}
