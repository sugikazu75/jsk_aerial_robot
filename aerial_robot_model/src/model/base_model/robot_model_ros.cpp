#include <aerial_robot_model/model/aerial_robot_model_ros.h>

namespace aerial_robot_model {
  RobotModelRos::RobotModelRos(ros_compat::NodeHandle nh, ros_compat::NodeHandle nhp):
    nh_(nh),
    nhp_(nhp),
    robot_model_loader_("aerial_robot_model", "aerial_robot_model::RobotModel")
  {
    br_ = ros_compat::makeBroadcaster<tf2_ros::TransformBroadcaster>(nh_);
    static_br_ = ros_compat::makeBroadcaster<tf2_ros::StaticTransformBroadcaster>(nh_);

    // rosparam
    nhp_.param("tf_prefix", tf_prefix_, std::string(""));

    // load robot model plugin
    std::string plugin_name;
    if(nh_.getParam("robot_model_plugin_name", plugin_name))
      {
        try
          {
            robot_model_ = ros_compat::createPluginInstance(robot_model_loader_, plugin_name);
          }
        catch(pluginlib::PluginlibException& ex)
          {
            ROS_COMPAT_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
          }
      }
    else
      {
        ROS_COMPAT_ERROR("can not find plugin rosparameter for robot model, use default class: aerial_robot_model::RobotModel");
        robot_model_ = ros_compat::makeShared<aerial_robot_model::RobotModel>();
      }

    if (robot_model_->isModelFixed()) {
      // broadcast static tf between root and CoG
      // refering: https://github.com/ros/robot_state_publisher/blob/rolling/src/robot_state_publisher.cpp#L130
      geometry_msgs_c::TransformStamped tf = robot_model_->getCog<geometry_msgs_c::TransformStamped>();
      tf.header.stamp = ros_compat::now();
      tf.header.frame_id = ros_compat::resolveFrame(tf_prefix_, robot_model_->getRootFrameName());
      tf.child_frame_id = ros_compat::resolveFrame(tf_prefix_, std::string("cog"));
      static_br_->sendTransform(tf);
    }
    else {
      joint_state_sub_ = nh_.subscribe("joint_states", 1, &RobotModelRos::jointStateCallback, this);
    }

    add_extra_module_service_ = ros_compat::advertiseService<aerial_robot_model_s::AddExtraModule>(
      nh_, "add_extra_module", &RobotModelRos::addExtraModuleCallback, this);
 }

  void RobotModelRos::jointStateCallback(const ros_compat::ConstPtr<sensor_msgs_c::JointState>& state)
  {
    joint_state_ = *state;
    robot_model_->updateRobotModel(*state);

    geometry_msgs_c::TransformStamped tf = robot_model_->getCog<geometry_msgs_c::TransformStamped>();
    tf.header = state->header;
    tf.header.frame_id = ros_compat::resolveFrame(tf_prefix_, robot_model_->getRootFrameName());
    tf.child_frame_id = ros_compat::resolveFrame(tf_prefix_, std::string("cog"));
    br_->sendTransform(tf);
  }

  bool RobotModelRos::addExtraModuleCallback(aerial_robot_model_s::AddExtraModule::Request &req, aerial_robot_model_s::AddExtraModule::Response &res)
  {
    switch(req.action)
      {
      case aerial_robot_model_s::AddExtraModule::Request::ADD:
        {
          geometry_msgs_c::TransformStamped ts;
          ts.transform = req.transform;
          KDL::Frame f = tf2::transformToKDL(ts);
          KDL::RigidBodyInertia rigid_body_inertia(req.inertia.m, KDL::Vector(req.inertia.com.x, req.inertia.com.y, req.inertia.com.z),
                                                   KDL::RotationalInertia(req.inertia.ixx, req.inertia.iyy,
                                                                          req.inertia.izz, req.inertia.ixy,
                                                                          req.inertia.ixz, req.inertia.iyz));
          res.status = robot_model_->addExtraModule(req.module_name, req.parent_link_name, f, rigid_body_inertia);
          return res.status;
          break;
        }
      case aerial_robot_model_s::AddExtraModule::Request::REMOVE:
        {
          res.status = robot_model_->removeExtraModule(req.module_name);
          return res.status;
          break;
        }
      default:
        {
          ROS_COMPAT_WARN("[extra module]: wrong action %d", req.action);
          return false;
          break;
        }
      }
    ROS_COMPAT_ERROR("[extra module]: should not reach here ");
    return false;
  }
} //namespace aerial_robot_model
