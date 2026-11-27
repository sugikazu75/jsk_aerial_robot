// -*- mode: c++ -*-

#include <gimbalrotor/gimbalrotor_navigation.h>
#include <aerial_robot_control/util/joy_parser.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

GimbalrotorNavigator::GimbalrotorNavigator() : BaseNavigator(), eq_cog_world_(false)
{
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
}

void GimbalrotorNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                      double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  target_baselink_rpy_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);  // to spinal
  final_target_baselink_rot_sub_ =
      nh_.subscribe("final_target_baselink_rot", 1, &GimbalrotorNavigator::targetBaselinkRotCallback, this);
  final_target_baselink_rpy_sub_ =
      nh_.subscribe("final_target_baselink_rpy", 1, &GimbalrotorNavigator::targetBaselinkRPYCallback, this);
  prev_rotation_stamp_ = ros::Time::now().toSec();

  // the joints at landing (none: only the body levelled)
  ros::NodeHandle navi_nh(nh_, "navigation");
  std::vector<std::string> names;
  std::vector<double> angles;
  navi_nh.param("landing_joint_names", names, std::vector<std::string>{});
  navi_nh.param("landing_joint_angles", angles, std::vector<double>{});
  if (names.size() != angles.size())
    ROS_ERROR("[Navigation] landing_joint_names and landing_joint_angles differ in size: the joints not moved");
  else
  {
    landing_joint_msg_.name = names;
    landing_joint_msg_.position = angles;
  }
  std::string topic;
  navi_nh.param("landing_joint_topic", topic, std::string("joints_ctrl"));
  navi_nh.param("landing_joint_tolerance", landing_joint_tolerance_, 0.085);
  landing_joint_pub_ = nh_.advertise<sensor_msgs::JointState>(topic, 1);
}

void GimbalrotorNavigator::update()
{
  BaseNavigator::update();
  landingProcess();
  baselinkRotationProcess();
}

bool GimbalrotorNavigator::isInflightState()
{
  return BaseNavigator::isInflightState() || getNaviState() == PRE_LAND_STATE;
}

void GimbalrotorNavigator::landingProcess()
{
  if ((getForceLandingFlag() || getNaviState() == LAND_STATE) && !level_flag_)
  {
    // the target attitude level, its yaw kept
    target_omega_.setValue(0, 0, 0);
    double r, p, y;
    tf::Matrix3x3(final_target_baselink_rot_).getRPY(r, p, y);
    final_target_baselink_rot_.setRPY(0, 0, y);
    if (getNaviState() == LAND_STATE)
    {
      // hold the height until level and the joints there
      setNaviState(PRE_LAND_STATE);
      setTeleopFlag(false);
      setTargetPosZ(estimator_->getState(State::Z_COG, estimate_mode_)[0]);
      setTargetVelZ(0);
      land_height_ = 0;  // updated again in the land state after this (see DragonNavigator)
      ROS_INFO("[Navigation] pre-land state: the body level and the joints to the landing angles");
    }
    landing_joint_stamp_ = 0;  // sent at once
    level_flag_ = true;
  }

  if (getNaviState() != PRE_LAND_STATE)
    return;

  // the joint angles of landing, again every 2 s against a lost message
  bool ready = true;
  if (!landing_joint_msg_.name.empty())
  {
    if (ros::Time::now().toSec() - landing_joint_stamp_ > 2.0)
    {
      landing_joint_msg_.header.stamp = ros::Time::now();
      landing_joint_pub_.publish(landing_joint_msg_);
      landing_joint_stamp_ = ros::Time::now().toSec();
    }
    const auto joints = robot_model_->kdlJointToMsg(robot_model_->getJointPositions());
    for (size_t i = 0; i < landing_joint_msg_.name.size(); i++)
    {
      const auto it = std::find(joints.name.begin(), joints.name.end(), landing_joint_msg_.name[i]);
      if (it != joints.name.end() &&
          std::fabs(joints.position[it - joints.name.begin()] - landing_joint_msg_.position[i]) > landing_joint_tolerance_)
        ready = false;
    }
  }
  double r, p, y;
  tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);
  if (std::fabs(r) > 0.01 || std::fabs(p) > 0.01)
    ready = false;

  if (ready)
  {
    ROS_INFO("[Navigation] level, the joints at the landing angles: back to the land state");
    setNaviState(LAND_STATE);
    setTeleopFlag(true);
  }
}

void GimbalrotorNavigator::reset()
{
  BaseNavigator::reset();
  level_flag_ = false;

  // reset SO3
  eq_cog_world_ = false;
  curr_target_baselink_rot_.setRPY(0, 0, 0);
  final_target_baselink_rot_.setRPY(0, 0, 0);
  KDL::Rotation rot;
  tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
  robot_model_->setCogDesireOrientation(rot);
}

void GimbalrotorNavigator::targetBaselinkRotCallback(const geometry_msgs::QuaternionStampedConstPtr& msg)
{
  tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_rot_);
  target_omega_.setValue(0, 0, 0);  // for sure to reset the target angular velocity

  // special process
  if (getTargetRPY().z() != 0)
  {
    curr_target_baselink_rot_.setRPY(0, 0, getTargetRPY().z());
    eq_cog_world_ = true;
  }
}

void GimbalrotorNavigator::targetBaselinkRPYCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  final_target_baselink_rot_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
  target_omega_.setValue(0, 0, 0);  // for sure to reset the target angular velocity
}

void GimbalrotorNavigator::naviCallback(const aerial_robot_msgs::FlightNavConstPtr& msg)
{
  BaseNavigator::naviCallback(msg);
  if (msg->roll_nav_mode == 2)
    setTargetRoll(msg->target_roll);
  if (msg->pitch_nav_mode == 2)
    setTargetPitch(msg->target_pitch);
}

void GimbalrotorNavigator::joyStickControl(const sensor_msgs::JoyConstPtr& joy_msg)
{
  // While L1 is held the sticks move the hand target of the whole-body IK (hanging_teleop.py): the
  // flight commands of the sticks (xy, z, yaw) are overwritten by 0, the buttons work as usual.
  // The parsed message is in the general (PS3) layout, which joyParse of the base passes through.
  sensor_msgs::Joy joy_cmd = joyParse(*joy_msg);
  if (joy_cmd.buttons.size() > JOY_BUTTON_REAR_LEFT_1 && joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] == 1)
  {
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = 0;
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = 0;
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = 0;
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = 0;
    BaseNavigator::joyStickControl(boost::make_shared<const sensor_msgs::Joy>(joy_cmd));
    return;
  }
  BaseNavigator::joyStickControl(joy_msg);
}

void GimbalrotorNavigator::baselinkRotationProcess()
{
  if (curr_target_baselink_rot_ == final_target_baselink_rot_)
    return;

  if (ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
  {
    tf::Quaternion delta_q = curr_target_baselink_rot_.inverse() * final_target_baselink_rot_;
    double angle = delta_q.getAngle();
    if (angle > M_PI)
      angle -= 2 * M_PI;

    if (fabs(angle) > baselink_rot_change_thresh_)
    {
      curr_target_baselink_rot_ *= tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);
    }
    else
      curr_target_baselink_rot_ = final_target_baselink_rot_;

    KDL::Rotation rot;
    tf::quaternionTFToKDL(curr_target_baselink_rot_, rot);
    robot_model_->setCogDesireOrientation(rot);

    // send to spinal
    spinal::DesireCoord msg;
    double r, p, y;
    tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);
    msg.roll = r;
    msg.pitch = p;
    msg.yaw = y;
    target_baselink_rpy_pub_.publish(msg);

    prev_rotation_stamp_ = ros::Time::now().toSec();
  }
}

void GimbalrotorNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_,
                   0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_,
                   0.1);  // the rate to pub baselink rotation command
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::GimbalrotorNavigator, aerial_robot_navigation::BaseNavigator);
