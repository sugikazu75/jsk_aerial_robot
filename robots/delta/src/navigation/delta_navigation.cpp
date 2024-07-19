#include <delta/navigation/delta_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

RollingNavigator::RollingNavigator():
  BaseNavigator(),
  poly_(11, agi::Vector<3>(0, 0, 1), 2)
{
  curr_target_baselink_quat_.setRPY(0, 0, 0);
  final_target_baselink_quat_.setRPY(0, 0, 0);
}

void RollingNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                  double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  rolling_robot_model_ = boost::dynamic_pointer_cast<RollingRobotModel>(robot_model_);

  desire_coord_pub_ = nh_.advertise<geometry_msgs::Quaternion>("desire_coordinate", 1);
  final_target_baselink_quat_sub_ = nh_.subscribe("final_target_baselink_quat", 1, &RollingNavigator::setFinalTargetBaselinkQuatCallback, this);
  final_target_baselink_rpy_sub_ = nh_.subscribe("final_target_baselink_rpy", 1, &RollingNavigator::setFinalTargetBaselinkRpyCallback, this);
  joy_sub_ = nh_.subscribe("joy", 1, &RollingNavigator::joyCallback, this);
  ground_navigation_mode_sub_ = nh_.subscribe("ground_navigation_command", 1, &RollingNavigator::groundNavigationModeCallback, this);
  ground_navigation_mode_pub_ = nh_.advertise<std_msgs::Int16>("ground_navigation_ack", 1);
  estimated_steep_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("estimated_steep", 1);
  prev_rotation_stamp_ = ros::Time::now().toSec();

  setPrevGroundNavigationMode(aerial_robot_navigation::NONE);
  setGroundNavigationMode(aerial_robot_navigation::FLYING_STATE);

  est_external_wrench_ = Eigen::VectorXd::Zero(6);
  est_external_wrench_cog_ = Eigen::VectorXd::Zero(6);
  estimated_steep_ = Eigen::VectorXd::Zero(2);

  controllers_reset_flag_ = false;
  baselink_rot_force_update_mode_ = false;

  curr_target_baselink_rpy_roll_ = 0.0;
  curr_target_baselink_rpy_pitch_ = 0.0;
  target_pitch_ang_vel_ = 0.0;
  target_yaw_ang_vel_ = 0.0;
  pitch_ang_vel_updating_ = false;
  yaw_ang_vel_updating_ = false;

  gimbal_reset_last_time_ = -1;
  gimbal_reset_index_ = 0;
  gimbal_reset_done_ = false;
}

void RollingNavigator::update()
{
  BaseNavigator::update();

  stateEstimation();

  rollingPlanner();

  gimbalPlanner();

  rosPublishProcess();
}

void RollingNavigator::reset()
{
  BaseNavigator::reset();

  curr_target_baselink_quat_.setRPY(0, 0, 0);
  final_target_baselink_quat_.setRPY(0, 0, 0);

  setPrevGroundNavigationMode(aerial_robot_navigation::NONE);
  setGroundNavigationMode(aerial_robot_navigation::FLYING_STATE);

  controllers_reset_flag_ = false;
  ground_trajectory_mode_ = false;
  poly_.reset();

  baselink_rot_force_update_mode_ = false;

  curr_target_baselink_rpy_roll_ = 0.0;
  curr_target_baselink_rpy_pitch_ = 0.0;
  target_pitch_ang_vel_ = 0.0;
  target_yaw_ang_vel_ = 0.0;
  pitch_ang_vel_updating_ = false;
  yaw_ang_vel_updating_ = false;

  gimbal_reset_last_time_ = -1;
  gimbal_reset_index_ = 0;
  gimbal_reset_done_ = false;
  for(int i = 0; i < robot_model_->getRotorNum(); i++)
    {
      rolling_robot_model_->setGimbalPlanningFlag(i, 0);
      rolling_robot_model_->setGimbalPlanningAngle(i, 0.0);
    }

  ROS_INFO_STREAM("[navigation] reset navigator");

}

void RollingNavigator::stateEstimation()
{
  /* steep estimation */
  if(getCurrentGroundNavigationMode() == aerial_robot_navigation::ROLLING_STATE)
    {
      tf::Quaternion cog2baselink_rot;
      tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
      tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
      double r, p, y;
      cog_rot.getRPY(r, p, y);

      KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
      KDL::Frame cog_alined;
      cog_alined.p = cog.p;
      cog_alined.M = cog.M;
      cog_alined.M.DoRotX(-r);
      cog_alined.M.DoRotY(-p);

      Eigen::Vector3d est_external_force_cog_alined = kdlToEigen(cog_alined.M).inverse() * kdlToEigen(cog.M) * est_external_wrench_cog_.head(3);
      Eigen::VectorXd estimated_steep = Eigen::VectorXd::Zero(2);
      estimated_steep(0) = atan2(est_external_force_cog_alined(1), sqrt(est_external_force_cog_alined(0) * est_external_force_cog_alined(0) + est_external_force_cog_alined(2) * est_external_force_cog_alined(2)));
      estimated_steep(1) = atan2(est_external_force_cog_alined(0), est_external_force_cog_alined(2));

      estimated_steep_
        = (steep_estimation_lpf_factor_ - 1.0) / steep_estimation_lpf_factor_ * estimated_steep_
        + 1.0 / steep_estimation_lpf_factor_ * estimated_steep;

      setTargetPitch(estimated_steep_(1));
    }

  /* contact point estimation */
  if(getCurrentGroundNavigationMode() == aerial_robot_navigation::ROLLING_STATE)
    {
      Eigen::Vector3d est_contact_point = aerial_robot_model::skew(est_external_wrench_cog_.head(3)) * est_external_wrench_cog_.tail(3) / (est_external_wrench_cog_.head(3).norm() * est_external_wrench_cog_.head(3).norm());
    }
}

void RollingNavigator::rollingPlanner()
{
  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
  double r, p, y;
  cog_rot.getRPY(r, p, y);

  switch(current_ground_navigation_mode_)
    {
    case aerial_robot_navigation::FLYING_STATE:
      {
        break;
      }

    case aerial_robot_navigation::STANDING_STATE:
      {
        /* if the robot has not started, generate trajectory every time */
        if(!(getNaviState() == aerial_robot_navigation::TAKEOFF_STATE || getNaviState() == aerial_robot_navigation::HOVER_STATE))
          {
            ground_trajectory_start_time_ = ros::Time::now().toSec();
            poly_.reset();
            poly_.scale(ground_trajectory_start_time_, ground_trajectory_duration_);
            poly_.addConstraint(ground_trajectory_start_time_, agi::Vector<3>(0.0, 0.0, 0.0));
            poly_.addConstraint(ground_trajectory_start_time_ + ground_trajectory_duration_, agi::Vector<3>(M_PI / 2.0, 0.0, 0.0));
            poly_.solve();
            ground_trajectory_mode_ = true;
            ROS_INFO_STREAM_THROTTLE(1.0, "[navigation] generate new standing trajectory from " << std::setprecision(12) << ground_trajectory_start_time_ << " to " << ground_trajectory_start_time_ + ground_trajectory_duration_);
          }

        /* evaluate current point in trajectory */
        agi::Vector<> baselink_roll_trajectory = agi::Vector<>::Zero(3);
        if(ground_trajectory_mode_)
          {
            poly_.eval(ros::Time::now().toSec(), baselink_roll_trajectory);

            if(ros::Time::now().toSec() > ground_trajectory_start_time_ + ground_trajectory_duration_)
              {
                ROS_INFO_STREAM("[navigation] finished trajectory tracking");
                ground_trajectory_mode_ = false;
              }
          }

        /* set trajectory or final state */
        Eigen::Matrix3d rot_mat;
        Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0);
        if(ground_trajectory_mode_)
          {
            setCurrentTargetBaselinkRpyRoll(baselink_roll_trajectory(0));
            rot_mat = Eigen::AngleAxisd(getCurrentTargetBaselinkRpyRoll(), b1);
            setTargetOmegaX(baselink_roll_trajectory(1));
            setTargetAngAccX(baselink_roll_trajectory(2));
          }
        else
          {
            rot_mat = Eigen::AngleAxisd(getCurrentTargetBaselinkRpyRoll(), b1);
            setTargetOmegaX(0.0);
            setTargetAngAccX(0.0);
          }

        /* set desire coordinate */
        KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
        double qx, qy, qz, qw;
        rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);

        setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));

        /* state transition to rolling state based on the baselink rotation */
        double baselink_roll = estimator_->getEuler(Frame::BASELINK, estimate_mode_).x();
        if(fabs(fabs(baselink_roll) - M_PI / 2.0) < standing_baselink_roll_converged_thresh_)
          {
            ROS_INFO_STREAM("[navigation] baselink roll " << baselink_roll << " is smaller than threshold " << standing_baselink_roll_converged_thresh_);
            setGroundNavigationMode(aerial_robot_navigation::ROLLING_STATE);
          }
        break;
      }

    case aerial_robot_navigation::ROLLING_STATE:
      {
        /* target rolling angle */
        double target_pitch;

        /* set current baselink pose as target until control is started */
        if(!(getNaviState() == aerial_robot_navigation::TAKEOFF_STATE || getNaviState() == aerial_robot_navigation::HOVER_STATE))
          {
            ground_trajectory_mode_ = false;

            tf::Matrix3x3 baselink_orientation = estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
            tf::Vector3 baselink_euler = estimator_->getEuler(Frame::BASELINK, estimate_mode_);

            ROS_INFO_STREAM_THROTTLE(1.0, "[navigation] set desire coordinate same as baselink roll: " << baselink_euler.x() << " pitch: " << baselink_euler.y());
            target_pitch = baselink_euler.y();
            setCurrentTargetBaselinkRpyPitch(target_pitch);
          }

        agi::Vector<> baselink_roll_trajectory = agi::Vector<>::Zero(3);
        if(ground_trajectory_mode_)
          {
            poly_.eval(ros::Time::now().toSec(), baselink_roll_trajectory);

            if(ros::Time::now().toSec() > ground_trajectory_start_time_ + ground_trajectory_duration_)
              {
                ROS_INFO_STREAM("[navigation] finished trajectory tracking");
                ground_trajectory_mode_ = false;
              }
          }

        /* calculate rolling pitch angle */
        if(!getPitchAngVelUpdating())
          {
            target_pitch = getCurrentTargetBaselinkRpyPitch();

            setCurrentTargetBaselinkRpyPitch(target_pitch);
            setTargetOmegaY(0);
          }
        else
          {
            double target_pitch_ang_vel = getTargetPitchAngVel();
            target_pitch = getCurrentTargetBaselinkRpyPitch();
            if(fabs(p - estimated_steep_(1)) < rolling_pitch_update_thresh_)
              {
                target_pitch += loop_du_ * target_pitch_ang_vel;
              }
            else
              {
                ROS_WARN_STREAM_THROTTLE(0.5, "[navigation] do not update target pitch because the pitch " << p - estimated_steep_(1) << " is  larger than thresh " << rolling_pitch_update_thresh_);
              }

            setCurrentTargetBaselinkRpyPitch(target_pitch);
            setTargetOmegaY(target_pitch_ang_vel);
          }

        /* set target controller target */
        if(ground_trajectory_mode_)
          {
            /* set pid term */
            setTargetOmegaX(baselink_roll_trajectory(1));
            setTargetAngAccX(baselink_roll_trajectory(2));

            /* set desire coordinate */
            setCurrentTargetBaselinkRpyRoll(baselink_roll_trajectory(0));
            Eigen::Matrix3d rot_mat;
            Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0);
            rot_mat = Eigen::AngleAxisd(getCurrentTargetBaselinkRpyRoll(), b1);
            KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
            double qx, qy, qz, qw;
            rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);

            setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
            setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
          }
        else
          {
            /* set pid term as 0 */
            setTargetOmegaX(0.0);
            setTargetAngAccX(0.0);

            /* set desire coordinate */
            double target_roll;
            if(!(getNaviState() == aerial_robot_navigation::TAKEOFF_STATE || getNaviState() == aerial_robot_navigation::HOVER_STATE))
              {
                tf::Vector3 baselink_euler = estimator_->getEuler(Frame::BASELINK, estimate_mode_);
                target_roll = baselink_euler.x();
                setCurrentTargetBaselinkRpyRoll(target_roll);
              }
            else
              {
                target_roll = getCurrentTargetBaselinkRpyRoll();
                setCurrentTargetBaselinkRpyRoll(target_roll);
              }
            Eigen::Matrix3d rot_mat;
            Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0), b2 = Eigen::Vector3d(0.0, 1.0, 0.0);
            rot_mat = Eigen::AngleAxisd(target_pitch, b2) * Eigen::AngleAxisd(target_roll, b1);
            KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
            double qx, qy, qz, qw;
            rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);

            setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
            setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
          }

        break;
      }

    case aerial_robot_navigation::DOWN_STATE:
      {
        Eigen::Matrix3d rot_mat;
        Eigen::Vector3d b1 = Eigen::Vector3d(1.0, 0.0, 0.0), b2 = Eigen::Vector3d(0.0, 1.0, 0.0);
        rot_mat = Eigen::AngleAxisd(getCurrentTargetBaselinkRpyPitch(), b2) * Eigen::AngleAxisd(getCurrentTargetBaselinkRpyRoll(), b1);
        double down_angle = std::min(std::max(0.0, down_mode_roll_anglvel_ * (ros::Time::now().toSec() - down_start_time_)), M_PI / 2.0);
        rot_mat = Eigen::AngleAxisd(-down_angle, b1) * rot_mat;

        KDL::Rotation rot_mat_kdl = eigenToKdl(rot_mat);
        double qx, qy, qz, qw;
        rot_mat_kdl.GetQuaternion(qx, qy, qz, qw);

        setCurrentTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        setFinalTargetBaselinkQuat(tf::Quaternion(qx, qy, qz, qw));
        break;
      }

    default:
      break;
    }
}

void RollingNavigator::gimbalPlanner()
{
  auto gimbal_planning_flag = rolling_robot_model_->getGimbalPlanningFlag();
  auto gimbal_planning_angle = rolling_robot_model_->getGimbalPlanningAngle();
  auto current_gimbal_angles = rolling_robot_model_->getCurrentGimbalAngles();

  if(current_ground_navigation_mode_ == aerial_robot_navigation::ROLLING_STATE)
    {
      if(!ground_trajectory_mode_ && ros::Time::now().toSec() > ground_trajectory_start_time_ + ground_trajectory_duration_ + 4.0) // wait for a little time
        {
          if(!gimbal_reset_done_)
            {
              if(!gimbal_planning_flag.at(gimbal_reset_index_) && ros::Time::now().toSec() - gimbal_reset_last_time_ > 1.0)
                {
                  rolling_robot_model_->setGimbalPlanningFlag(gimbal_reset_index_, 1);
                  rolling_robot_model_->setGimbalPlanningAngle(gimbal_reset_index_, M_PI / 2.0);
                  return;
                }
              if(fabs(gimbal_planning_angle.at(gimbal_reset_index_) - current_gimbal_angles.at(gimbal_reset_index_)) < gimbal_planning_converged_thresh_)
                {
                  rolling_robot_model_->setGimbalPlanningFlag(gimbal_reset_index_, 0);
                  ROS_INFO_STREAM("[navigation] planning for gimbal" << gimbal_reset_index_ << " is converged. target: " << gimbal_planning_angle.at(gimbal_reset_index_) << ", thresh: " << gimbal_planning_converged_thresh_);
                  gimbal_reset_index_++;
                  gimbal_reset_last_time_ = ros::Time::now().toSec();

                  if(gimbal_reset_index_ == rolling_robot_model_->getRotorNum())
                    {
                      gimbal_reset_done_ = true;
                      for(int i = 0; i < rolling_robot_model_->getRotorNum(); i++)
                        {
                          if(fabs(current_gimbal_angles.at(i) - M_PI /2.0) > M_PI / 2.0)
                            {
                              gimbal_reset_done_ = false;
                              gimbal_reset_index_ = i;
                              gimbal_reset_last_time_ = ros::Time::now().toSec();
                            }
                        }
                      if(gimbal_reset_done_) ROS_INFO_STREAM("[navigation] finished gimbal angle reset");
                    }
                }
            }
        }
    }
  else
    {
      for(int i = 0; i < robot_model_->getRotorNum(); i++)
        {
          rolling_robot_model_->setGimbalPlanningFlag(i, 0);
          rolling_robot_model_->setGimbalPlanningAngle(i, 0.0);
        }
    }
}

void RollingNavigator::rosPublishProcess()
{
  baselinkRotationProcess();
  groundModeProcess();

  std_msgs::Float64MultiArray estimated_steep_msg;
  for(int i = 0; i < estimated_steep_.size(); i++)
    {
      estimated_steep_msg.data.push_back(estimated_steep_(i));
    }
  estimated_steep_pub_.publish(estimated_steep_msg);
}

void RollingNavigator::baselinkRotationProcess()
{
  if(ros::Time::now().toSec() - prev_rotation_stamp_ > baselink_rot_pub_interval_)
    {
      tf::Quaternion delta_q = curr_target_baselink_quat_.inverse() * final_target_baselink_quat_;
      double angle = delta_q.getAngle();
      if (angle > M_PI) angle -= 2 * M_PI;

      if(fabs(angle) > baselink_rot_change_thresh_)
        {
          curr_target_baselink_quat_ *= tf::Quaternion(delta_q.getAxis(), fabs(angle) / angle * baselink_rot_change_thresh_);
        }
      else
        curr_target_baselink_quat_ = final_target_baselink_quat_;

      KDL::Rotation rot;
      tf::quaternionTFToKDL(curr_target_baselink_quat_, rot);

      /* set to robot model */
      robot_model_->setCogDesireOrientation(rot);

      /* send to spinal */
      geometry_msgs::Quaternion msg;
      msg.x = curr_target_baselink_quat_.x();
      msg.y = curr_target_baselink_quat_.y();
      msg.z = curr_target_baselink_quat_.z();
      msg.w = curr_target_baselink_quat_.w();
      desire_coord_pub_.publish(msg);

      prev_rotation_stamp_ = ros::Time::now().toSec();
    }
}

void RollingNavigator::groundModeProcess()
{
  std_msgs::Int16 msg;
  msg.data = current_ground_navigation_mode_;
  ground_navigation_mode_pub_.publish(msg);
}

void RollingNavigator::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle navi_nh(nh_, "navigation");

  getParam<double>(navi_nh, "baselink_rot_change_thresh", baselink_rot_change_thresh_, 0.02);  // the threshold to change the baselink rotation
  getParam<double>(navi_nh, "baselink_rot_pub_interval", baselink_rot_pub_interval_, 0.1); // the rate to pub baselink rotation command
  getParam<double>(navi_nh, "joy_stick_deadzone", joy_stick_deadzone_, 0.2);
  getParam<double>(navi_nh, "rolling_max_pitch_ang_vel", rolling_max_pitch_ang_vel_, 0.0);
  getParam<double>(navi_nh, "rolling_max_yaw_ang_vel", rolling_max_yaw_ang_vel_, 0.0);
  getParam<double>(navi_nh, "down_mode_roll_angvel", down_mode_roll_anglvel_, 0.0);
  getParam<double>(navi_nh, "standing_baselink_roll_converged_thresh", standing_baselink_roll_converged_thresh_, 0.0);
  getParam<double>(navi_nh, "rolling_pitch_update_thresh", rolling_pitch_update_thresh_, 0.0);
  getParam<double>(navi_nh, "ground_trajectory_duration", ground_trajectory_duration_, 0.0);
  getParam<double>(navi_nh, "gimbal_planning_converged_thresh", gimbal_planning_converged_thresh_, 0.1);
  getParam<double>(navi_nh, "steep_estimation_lpf_factor", steep_estimation_lpf_factor_, 20.0);

}

void RollingNavigator::setGroundNavigationMode(int state)
{
  if(state != current_ground_navigation_mode_)
    {
      ROS_WARN_STREAM("[navigation] switch to " << indexToGroundNavigationModeString(state));
    }
  else
    {
      return;
    }

  if(state == aerial_robot_navigation::FLYING_STATE)
    {
      setTargetXyFromCurrentState();

      ros::NodeHandle navi_nh(nh_, "navigation");
      double target_z;
      getParam<double>(navi_nh, "takeoff_height", target_z, 1.0);
      setTargetPosZ(target_z);

      setTargetYawFromCurrentState();

      controllers_reset_flag_ = true;
    }

  if(state == aerial_robot_navigation::STANDING_STATE)
    {
      ground_trajectory_start_time_ = ros::Time::now().toSec();
      poly_.reset();
      poly_.scale(ground_trajectory_start_time_, ground_trajectory_duration_);
      poly_.addConstraint(ground_trajectory_start_time_, agi::Vector<3>(0.0, 0.0, 0.0));
      poly_.addConstraint(ground_trajectory_start_time_ + ground_trajectory_duration_, agi::Vector<3>(M_PI / 2.0, 0.0, 0.0));
      poly_.solve();
      ground_trajectory_mode_ = true;

      Eigen::Vector3d cog_pos;
      // Eigen::Matrix3d cog_orientation = 
      vectorTFToEigen(estimator_->getPos(Frame::COG, estimate_mode_), cog_pos);
      KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
      KDL::Frame contact_point = rolling_robot_model_->getContactPoint<KDL::Frame>();
      
      
    }

  if(state == aerial_robot_navigation::ROLLING_STATE)
    {
    }

  if(state == aerial_robot_navigation::DOWN_STATE)
    {
      down_start_time_ = ros::Time::now().toSec();
      ROS_INFO_STREAM("[navigation] down start at: " << down_start_time_);
    }

  current_ground_navigation_mode_ = state;
}

void RollingNavigator::groundNavigationModeCallback(const std_msgs::Int16Ptr & msg)
{
  ROS_WARN_STREAM("[navigation] ground navigation command callback: switch mode to " << indexToGroundNavigationModeString(msg->data));
  setGroundNavigationMode(msg->data);
}

void RollingNavigator::setFinalTargetBaselinkQuatCallback(const geometry_msgs::QuaternionStampedConstPtr & msg)
{
  tf::Quaternion final_target_baselink_quat;
  tf::quaternionMsgToTF(msg->quaternion, final_target_baselink_quat);
}

void RollingNavigator::setFinalTargetBaselinkRpyCallback(const geometry_msgs::Vector3StampedConstPtr & msg)
{
  ROS_WARN_STREAM("[navigation] baselink rotation callback. RPY: " << msg->vector.x << " " << msg->vector.y << " " << msg->vector.z);
  final_target_baselink_quat_.setRPY(msg->vector.x, msg->vector.y, msg->vector.z);
}

void RollingNavigator::joyCallback(const sensor_msgs::JoyConstPtr & joy_msg)
{
  sensor_msgs::Joy joy_cmd = (*joy_msg);

  /* change ground navigation state */
  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == 1.0 && current_ground_navigation_mode_ != aerial_robot_navigation::STANDING_STATE)
    {
      ROS_INFO_STREAM("[joy] change to " << indexToGroundNavigationModeString(aerial_robot_navigation::STANDING_STATE));
      setGroundNavigationMode(aerial_robot_navigation::STANDING_STATE);
    }

  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == -1.0 && current_ground_navigation_mode_ != aerial_robot_navigation::ROLLING_STATE)
    {
      ROS_INFO_STREAM("[joy] change to " << indexToGroundNavigationModeString(aerial_robot_navigation::ROLLING_STATE));
      setGroundNavigationMode(aerial_robot_navigation::ROLLING_STATE);
    }

  if(joy_cmd.buttons[PS4_BUTTON_REAR_RIGHT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1.0 && current_ground_navigation_mode_ != aerial_robot_navigation::FLYING_STATE)
    {
      ROS_INFO_STREAM("[joy] change to " << indexToGroundNavigationModeString(aerial_robot_navigation::FLYING_STATE));
      setGroundNavigationMode(aerial_robot_navigation::FLYING_STATE);
    }

  if(joy_cmd.buttons[PS4_BUTTON_REAR_RIGHT_1] && joy_cmd.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1.0 && current_ground_navigation_mode_ != aerial_robot_navigation::DOWN_STATE)
    {
      ROS_INFO_STREAM("[joy] change to " << indexToGroundNavigationModeString(aerial_robot_navigation::DOWN_STATE));
      setGroundNavigationMode(aerial_robot_navigation::DOWN_STATE);
    }

  /* set target angular velocity around yaw based on R-stick hilizontal */
  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && fabs(joy_cmd.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS]) > joy_stick_deadzone_)
    {
      target_yaw_ang_vel_ = rolling_max_yaw_ang_vel_ * joy_cmd.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS];

      addTargetYaw(loop_du_ * target_yaw_ang_vel_);
      setTargetOmegaZ(target_yaw_ang_vel_);

      yaw_ang_vel_updating_ = true;
    }
  else
    {
      if(yaw_ang_vel_updating_)
        {
          target_yaw_ang_vel_ = 0.0;

          tf::Quaternion cog2baselink_rot;
          tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
          tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) * tf::Matrix3x3(cog2baselink_rot).inverse();
          double r, p, y;
          cog_rot.getRPY(r, p, y);

          setTargetYaw(y);
          setTargetOmegaZ(0.0);

          yaw_ang_vel_updating_ = false;
        }
    }

  /* set target angular velocity around pitch based on L-stick vertical */
  if(joy_cmd.buttons[PS4_BUTTON_REAR_LEFT_1] && fabs(joy_cmd.axes[PS4_AXIS_STICK_LEFT_UPWARDS]) > joy_stick_deadzone_)
    {
      target_pitch_ang_vel_ = rolling_max_pitch_ang_vel_ * joy_cmd.axes[PS4_AXIS_STICK_LEFT_UPWARDS];
      pitch_ang_vel_updating_ = true;
    }
  else
    {
      if(pitch_ang_vel_updating_)
        {
          target_pitch_ang_vel_ = 0.0;
          pitch_ang_vel_updating_ = false;
        }
    }
}

std::string RollingNavigator::indexToGroundNavigationModeString(int index)
{
  switch(index){
  case aerial_robot_navigation::FLYING_STATE:
    return "FLYING_STATE";
    break;
  case aerial_robot_navigation::STANDING_STATE:
    return "STANDING_STATE";
    break;
  case aerial_robot_navigation::ROLLING_STATE:
    return "ROLLING_STATE";
        break;
  case aerial_robot_navigation::DOWN_STATE:
    return "DOWN_STATE";
  default:
    return "NONE";
        break;
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::RollingNavigator, aerial_robot_navigation::BaseNavigator);
