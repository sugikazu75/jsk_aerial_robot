#include <rolling/control/rolling_controller.h>


using namespace aerial_robot_model;
using namespace aerial_robot_control;

RollingController::RollingController():
  PoseLinearController(),
  torque_allocation_matrix_inv_pub_stamp_(0)
{
}

void RollingController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                   double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  rolling_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::RollingNavigator>(navigator_);
  rolling_robot_model_ = boost::dynamic_pointer_cast<RollingRobotModel>(robot_model_);
  robot_model_for_control_ = boost::make_shared<aerial_robot_model::RobotModel>();

  rotor_tilt_.resize(motor_num_);
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_, 0);

  target_wrench_acc_cog_.resize(6);
  full_lambda_trans_.resize(2 * motor_num_);
  full_lambda_rot_.resize(2 * motor_num_);
  full_lambda_all_.resize(2 * motor_num_);
  full_q_mat_.resize(6, 2 * motor_num_);
  full_q_mat_inv_.resize(2 * motor_num_, 6);

  q_mat_.resize(6, motor_num_);
  q_mat_inv_.resize(motor_num_, 6);

  rosParamInit();

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  target_wrench_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_wrench_acc_cog", 1);
  wrench_allocation_matrix_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix", 1);
  ground_mode_sub_ = nh_.subscribe("ground_mode", 1, &RollingController::groundModeCallback, this);
  joint_state_sub_ = nh_.subscribe("joint_states", 1, &RollingController::jointStateCallback, this);

  ground_mode_ = 0;
  gain_updated_ = false;
}

void RollingController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();
  ground_mode_ = 0;
  gain_updated_ = false;
}

void RollingController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle base_nh(nh_, "aerial_robot_base_node");
  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
  getParam<double>(control_nh, "allocation_refine_threshold", allocation_refine_threshold_, 0.01);
  getParam<int>(control_nh, "allocation_refine_max_iteration", allocation_refine_max_iteration_, 1);
  getParam<double>(control_nh, "initial_roll_tilt", initial_roll_tilt_, 0.0);

  getParam<double>(nh_, "circle_radius", circle_radius_, 0.5);
  rolling_robot_model_->setCircleRadius(circle_radius_);

  getParam<string>(base_nh, "tf_prefix", tf_prefix_, std::string(""));

  double rotor_tilt1;
  double rotor_tilt2;
  double rotor_tilt3;
  getParam<double>(control_nh, "rotor_tilt1", rotor_tilt1, 0.0);
  rotor_tilt_.at(0) = rotor_tilt1;
  getParam<double>(control_nh, "rotor_tilt2", rotor_tilt2, 0.0);
  rotor_tilt_.at(1) = rotor_tilt2;
  getParam<double>(control_nh, "rotor_tilt3", rotor_tilt3, 0.0);
  rotor_tilt_.at(2) = rotor_tilt3;
}

void RollingController::controlCore()
{
  if(!gain_updated_)
    {
      if(ground_mode_)
        {
          ros::NodeHandle ground_control_nh(nh_, "ground_controller");
          ros::NodeHandle xy_nh(ground_control_nh, "xy");
          ros::NodeHandle x_nh(ground_control_nh, "x");
          ros::NodeHandle y_nh(ground_control_nh, "y");
          ros::NodeHandle z_nh(ground_control_nh, "z");
          ros::NodeHandle roll_pitch_nh(ground_control_nh, "roll_pitch");
          ros::NodeHandle roll_nh(ground_control_nh, "roll");
          ros::NodeHandle pitch_nh(ground_control_nh, "pitch");
          ros::NodeHandle yaw_nh(ground_control_nh, "yaw");
        }
      gain_updated_ = true;
    }

  flightControl();
  // groundControl();

  // std::vector<Eigen::Vector3d> rotor_origin = rolling_robot_model_->getRotorsOriginFromContactPoint<Eigen::Vector3d>();
  // for(int i = 0; i < motor_num_; i++)
  //   {
  //     std::cout << rotor_origin.at(i) << std::endl;
  //   }
  // std::cout << std::endl;
  // std::vector<Eigen::Vector3d> rotor_normal = rolling_robot_model_->getRotorsNormalFromContactPoint<Eigen::Vector3d>();
  // for(int i = 0; i < motor_num_; i++)
  //   {
  //     std::cout << rotor_normal.at(i) << std::endl;
  //   }
  // std::cout << std::endl;
  // Eigen::Matrix3d inertia_cog = robot_model_->getInertia<Eigen::Matrix3d>();
  // Eigen::Matrix3d inertia_cp = rolling_robot_model_->getInertiaContactPoint<Eigen::Matrix3d>();
  // std::cout << inertia_cog << std::endl;
  // std::cout << inertia_cp << std::endl;
  // std::cout << std::endl;
}

void RollingController::flightControl()
{
  PoseLinearController::controlCore();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  target_wrench_acc_cog_ = target_wrench_acc_cog;

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  if(navigator_->getForceLandingFlag() && target_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;

  RollingController::wrenchAllocationFromCog();

  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  for (unsigned int i = 0; i < motor_num_; i++)
    {
      if(q_mat_inv_(i, YAW) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW);
    }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
}

void RollingController::groundControl()
{
  PoseLinearController::controlCore();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 euler = estimator_->getEuler(Frame::COG, estimate_mode_);
  pid_controllers_.at(ROLL).setIGain(0.0);

  tf::Vector3 rot = tf::Vector3(initial_roll_tilt_, 0, 0);
  rolling_navigator_->setFinalTargetBaselinkRot(rot);
  tf::Vector3 curr_target_baselink_rot = rolling_navigator_->getCurrTargetBaselinkRot();
  double curr_target_baselink_rot_roll = curr_target_baselink_rot.x();
  navigator_->setTargetPosZ(circle_radius_ * sin(curr_target_baselink_rot_roll));

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  double mass = robot_model_->getMass();
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::Vector3d target_force_cog = mass * Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  
  Eigen::MatrixXd inertia = robot_model_for_control_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d target_torque = inertia * Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  
  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  double ff_torque_x = (mass * aerial_robot_estimation::G - target_force_cog(2)) * cos(curr_target_baselink_rot_roll) + target_force_cog(1) * sin(curr_target_baselink_rot_roll);
  double ff_torque_y = -target_force_cog(0) * sin(curr_target_baselink_rot_roll);
  double ff_torque_z = target_force_cog(0) * cos(curr_target_baselink_rot_roll);
  Eigen::Vector3d ff_torque = circle_radius_ * Eigen::Vector3d(ff_torque_x, ff_torque_y, ff_torque_z);
  std::cout << ff_torque(0) << " " << ff_torque(1) << " " << ff_torque(2) << " " << std::endl;
  std::cout << std::endl;

  target_torque = target_torque + ff_torque;

  target_wrench_acc_cog_.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());
  target_wrench_acc_cog_.tail(3) = inertia.inverse() * target_torque;

  if(navigator_->getForceLandingFlag() && target_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;

  RollingController::wrenchAllocationFromCog();

  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  for (unsigned int i = 0; i < motor_num_; i++)
    {
      if(q_mat_inv_(i, YAW) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW);
    }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
}

void RollingController::wrenchAllocationFromCog()
{
  /* iterarively calcurate rotor origin */
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation);
  KDL::JntArray gimbal_processed_joint = robot_model_->getJointPositions();
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);

  for(int j = 0; j < allocation_refine_max_iteration_; j++)
    {
      /* calculate normal allocation */
      Eigen::MatrixXd wrench_matrix = Eigen::MatrixXd::Zero(6, 3 * motor_num_);
      Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
      wrench_map.block(0, 0, 3, 3) =  Eigen::MatrixXd::Identity(3, 3);

      int last_col = 0;
      std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();
      for(int i = 0; i < motor_num_; i++)
        {
          wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));
          wrench_matrix.middleCols(last_col, 3) = wrench_map;

          last_col += 3;
        }

      Eigen::Matrix3d inertia_inv = robot_model_for_control_->getInertia<Eigen::Matrix3d>().inverse();
      double mass_inv = 1 / robot_model_->getMass();
      wrench_matrix.topRows(3) = mass_inv * wrench_matrix.topRows(3);
      wrench_matrix.bottomRows(3) = inertia_inv * wrench_matrix.bottomRows(3);

      /* calculate masked and integrated rotaion matrix */
      Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, 2 * motor_num_);
      const auto links_rotation_from_cog = rolling_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
      Eigen::MatrixXd mask(3, 2);
      mask << 0, 0, 1, 0, 0, 1;
      for(int i = 0; i < motor_num_; i++)
        {
          integrated_rot.block(3 * i, 2 * i, 3, 2) = links_rotation_from_cog.at(i) * mask;
        }

      /* calculate integarated allocation */
      full_q_mat_ = wrench_matrix * integrated_rot;
      full_q_mat_inv_ = aerial_robot_model::pseudoinverse(full_q_mat_);

      /* actuator mapping */
      full_lambda_trans_ = full_q_mat_inv_.leftCols(3) * target_wrench_acc_cog_.head(3);
      full_lambda_rot_ = full_q_mat_inv_.rightCols(3) * target_wrench_acc_cog_.tail(3);
      full_lambda_all_ = full_lambda_trans_ + full_lambda_rot_;

      last_col = 0;
      for(int i = 0; i < motor_num_; i++)
        {
          Eigen::VectorXd full_lambda_trans_i = full_lambda_trans_.segment(last_col, 2);
          Eigen::VectorXd full_lambda_all_i = full_lambda_all_.segment(last_col, 2);
          target_gimbal_angles_.at(i) = atan2(-full_lambda_all_i(0), full_lambda_all_i(1));
          target_base_thrust_.at(i) = full_lambda_trans_i.norm() / fabs(cos(rotor_tilt_.at(i)));

          last_col += 2;
        }

      /* update robot model by calculated gimbal angle */
      const auto& joint_index_map = robot_model_->getJointIndexMap();
      gimbal_processed_joint = robot_model_for_control_->getJointPositions();
      for(int i = 0; i < motor_num_; i++)
        {
          std::string s = std::to_string(i + 1);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s)->second) = target_gimbal_angles_.at(i);
        }

      std::vector<Eigen::Vector3d> prev_rotors_origin_from_cog = rotors_origin_from_cog;
      robot_model_for_control_->updateRobotModel(gimbal_processed_joint);
      rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();

      double max_diff = 1e-6;
      for(int i = 0; i < motor_num_; i++)
        {
          double diff = (rotors_origin_from_cog.at(i) - prev_rotors_origin_from_cog.at(i)).norm();
          if(max_diff < diff) max_diff = diff;
        }

      if(control_verbose_)
        {
          // ROS_DEBUG_STREAM("refine rotor origin in control: iteration "<< j+1 << ", max_diff: " << max_diff);  // din't work
          ROS_WARN_STREAM("refine rotor origin in control: iteration "<< j+1 << ", max_diff: " << max_diff);      // worked
        }
      if(max_diff < allocation_refine_threshold_)
        {
          break;
        }
      if(j == allocation_refine_max_iteration_ - 1)
        {
          ROS_WARN_STREAM("refine rotor origin in control: can not converge in iteration " << j+1 << " max_diff: " << max_diff);
        }
    }  /* iterarively calcurate rotor origin */


  /* calculate allocation matrix for realtime control */
  q_mat_ = robot_model_for_control_->calcWrenchMatrixOnCoG();
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);
}


void RollingController::sendCmd()
{
  PoseLinearController::sendCmd();

  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for(int i = 0; i < motor_num_; i++){
    gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
  }
  gimbal_control_pub_.publish(gimbal_control_msg);

  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < full_lambda_all_.size(); i++)
    {
      target_vectoring_force_msg.data.push_back(full_lambda_all_(i));
    }
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

  std_msgs::Float32MultiArray target_wrench_acc_cog_msg;
  for(int i = 0; i < target_wrench_acc_cog_.size(); i++)
    {
      target_wrench_acc_cog_msg.data.push_back(target_wrench_acc_cog_(i));
    }
  target_wrench_acc_cog_pub_.publish(target_wrench_acc_cog_msg);

  aerial_robot_msgs::WrenchAllocationMatrix wrench_allocation_matrix_msg;
  for(int i = 0; i < q_mat_.cols(); i++)
    {
      wrench_allocation_matrix_msg.f_x.push_back(q_mat_(0, i));
      wrench_allocation_matrix_msg.f_y.push_back(q_mat_(1, i));
      wrench_allocation_matrix_msg.f_z.push_back(q_mat_(2, i));
      wrench_allocation_matrix_msg.t_x.push_back(q_mat_(3, i));
      wrench_allocation_matrix_msg.t_y.push_back(q_mat_(4, i));
      wrench_allocation_matrix_msg.t_z.push_back(q_mat_(5, i));
    }
  wrench_allocation_matrix_pub_.publish(wrench_allocation_matrix_msg);

  sendFourAxisCommand();

  sendTorqueAllocationMatrixInv();
}

void RollingController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

void RollingController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
      Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
      if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
        ROS_ERROR("Torque Allocation Matrix overflow");
      for (unsigned int i = 0; i < motor_num_; i++)
        {
          torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
        }
      torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
    }
}

void RollingController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void RollingController::groundModeCallback(const std_msgs::Int16Ptr & msg)
{
  int prev_ground_mode = ground_mode_;
  ground_mode_ = msg->data;
  ROS_INFO_STREAM("[control] changed mode from " << prev_ground_mode << " to " << ground_mode_);
  gain_updated_ = false;
}

void RollingController::jointStateCallback(const sensor_msgs::JointStateConstPtr & state)
{
  sensor_msgs::JointState joint_state = *state;
  geometry_msgs::TransformStamped tf = rolling_robot_model_->getContactPoint<geometry_msgs::TransformStamped>();
  tf.header = state->header;
  tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  tf.child_frame_id = tf::resolve(tf_prefix_, std::string("cp"));
  br_.sendTransform(tf);
}



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::RollingController, aerial_robot_control::ControlBase);