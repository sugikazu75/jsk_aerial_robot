#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

DeltaController::DeltaController():
  ControlBase()
{
}

void DeltaController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                 boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                 double ctrl_loop_rate)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  ROS_ERROR("initialize");
  rosParamInit();

  dt_ = holizon_ / (double) n_grid_;
  ROS_WARN_STREAM("[MPC] dt: " << dt_);

  target_base_thrust_.resize(motor_num_);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  torque_allocation_matrix_inv_pub_stamp_ = 0.0;
  torque_allocation_matrix_inv_pub_interval_ = 0.05;

  solution_ = Eigen::VectorXd::Zero(nx_ + n_grid_ * (nx_ + nu_));
  x_ref_eigen_ = Eigen::VectorXd::Zero(nx_);
  x_ref_ = eigenVectorToCasadiDm(x_ref_eigen_);
  lam_x_ = eigenVectorToCasadiDm(Eigen::VectorXd::Zero(nx_ + n_grid_ * (nx_ + nu_)));
  lam_g_ = eigenVectorToCasadiDm(Eigen::VectorXd::Zero(n_grid_ * nx_));

  visualize_thread_ = std::thread(boost::bind(&DeltaController::visualizeFunc, this));

}

void DeltaController::reset()
{
  ControlBase::reset();

  solution_ = Eigen::VectorXd::Zero(nx_ + n_grid_ * (nx_ + nu_));
  x_ref_eigen_ = Eigen::VectorXd::Zero(nx_);
  x_ref_ = eigenVectorToCasadiDm(x_ref_eigen_);
  lam_x_ = eigenVectorToCasadiDm(Eigen::VectorXd::Zero(nx_ + n_grid_ * (nx_ + nu_)));
  lam_g_ = eigenVectorToCasadiDm(Eigen::VectorXd::Zero(n_grid_ * nx_));

  candidate_yaw_term_ = 0.0;

  setAttitudeGains();

  ROS_ERROR("reset");
}

void DeltaController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<double>(control_nh, "holizon", holizon_, 0.0);
  getParam<int>(control_nh, "n_grid", n_grid_, 0);
  getParam<int>(control_nh, "nx", nx_, 0);
  getParam<int>(control_nh, "nu", nu_, 0);

  double q_rx, q_ry, q_rz, q_vx, q_vy, q_vz, q_alpha_x, q_alpha_y, q_alpha_z, q_omega_x, q_omega_y, q_omega_z;
  getParam<double>(control_nh, "q_rx", q_rx, 0.0);
  getParam<double>(control_nh, "q_ry", q_ry, 0.0);
  getParam<double>(control_nh, "q_rz", q_rz, 0.0);

  getParam<double>(control_nh, "q_vx", q_vx, 0.0);
  getParam<double>(control_nh, "q_vy", q_vy, 0.0);
  getParam<double>(control_nh, "q_vz", q_vz, 0.0);

  getParam<double>(control_nh, "q_alpha_x", q_alpha_x, 0.0);
  getParam<double>(control_nh, "q_alpha_y", q_alpha_y, 0.0);
  getParam<double>(control_nh, "q_alpha_z", q_alpha_z, 0.0);

  getParam<double>(control_nh, "q_omega_x", q_omega_x, 0.0);
  getParam<double>(control_nh, "q_omega_y", q_omega_y, 0.0);
  getParam<double>(control_nh, "q_omega_z", q_omega_z, 0.0);

  Eigen::VectorXd x_weight_vec = Eigen::VectorXd(nx_);
  x_weight_vec << q_rx, q_ry, q_rz, q_vx, q_vy, q_vz, q_alpha_x, q_alpha_y, q_alpha_z, q_omega_x, q_omega_y, q_omega_z;
  x_weight_mat_ = x_weight_vec.asDiagonal();

  double u_thrust;
  getParam<double>(control_nh, "u_thrust", u_thrust, 0.0);
  u_weight_mat_ = (Eigen::VectorXd::Ones(nu_) * u_thrust).asDiagonal();

  candidate_yaw_term_ = 0.0;
  getParam<double>(control_nh, "yaw_p_gain", yaw_p_gain_, 0.0);

  ros::NodeHandle base_nh(nh_, "aerial_robot_base_node");
  getParam<string>(base_nh, "tf_prefix", tf_prefix_, std::string(""));

}

bool DeltaController::update()
{
  if(!ControlBase::update()) return false;

  controlCore();
  sendCmd();

  return true;
}

casadi::MX DeltaController::dynamics(casadi::MX x, casadi::MX u)
{
  casadi::MX x_dot = casadi::MX::zeros(nx_);
  /* vel */
  x_dot(0) = x(3);
  x_dot(1) = x(4);
  x_dot(2) = x(5);

  /* omega */
  x_dot(6) = x(9);
  x_dot(7) = x(10);
  x_dot(8) = x(11);

  /* rotation matrix */
  casadi::MX rot_mat = casadi::MX::zeros(3, 3);
  int offset = 3;
  rot_mat(0, 0) =  cos(x(YAW + offset))   * cos(x(PITCH + offset));
  rot_mat(0, 1) =  cos(x(YAW + offset))   * sin(x(PITCH + offset)) * sin(x(ROLL + offset))  - cos(x(ROLL + offset))  * sin(x(YAW + offset));
  rot_mat(0, 2) =  sin(x(YAW + offset))   * sin(x(ROLL + offset))  + cos(x(YAW + offset))   * cos(x(ROLL + offset))  * sin(x(PITCH + offset));
  rot_mat(1, 0) =  cos(x(PITCH + offset)) * sin(x(YAW + offset));
  rot_mat(1, 1) =  cos(x(YAW + offset))   * cos(x(ROLL + offset))  + sin(x(YAW + offset))   * sin(x(PITCH + offset)) * sin(x(ROLL + offset));
  rot_mat(1, 2) =  cos(x(ROLL + offset))  * sin(x(YAW + offset))   * sin(x(PITCH + offset)) - cos(x(YAW + offset))   * sin(x(ROLL + offset));
  rot_mat(2, 0) = -sin(x(PITCH + offset));
  rot_mat(2, 1) =  cos(x(PITCH + offset)) * sin(x(ROLL + offset));
  rot_mat(2, 2) =  cos(x(PITCH + offset)) * cos(x(ROLL + offset));

  Eigen::MatrixXd wrench_matrix = robot_model_->calcWrenchMatrixOnCoG();

  /* force to acc */
  casadi::DM q_trans = eigenMatrixToCasadiDM(wrench_matrix.topRows(3));
  casadi::MX exerted_force_cog = casadi::MX::zeros(3);
  for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < motor_num_; j++)
        {
          exerted_force_cog(i) += q_trans(i, j) * u(j);
        }
    }
  casadi::MX exerted_acc_w = rot_mat * exerted_force_cog / robot_model_->getMass();
  x_dot(3) = exerted_acc_w(0);
  x_dot(4) = exerted_acc_w(1);
  x_dot(5) = exerted_acc_w(2);

  /* force to ang acc */
  casadi::DM inertia_inv = eigenMatrixToCasadiDM(robot_model_->getInertia<Eigen::Matrix3d>().inverse());
  casadi::DM q_rot = eigenMatrixToCasadiDM(wrench_matrix.bottomRows(3));
  casadi::MX exerted_torque = casadi::MX::zeros(3);
  for(int i = 0; i < 3; i++)
    {
      for(int j = 0; j < motor_num_; j++)
        {
          exerted_torque(i) += q_rot(i, j) * u(j);
        }
    }
  casadi::MX exerted_ang_acc = inertia_inv * exerted_torque;
  x_dot(9) = exerted_ang_acc(0);
  x_dot(10) = exerted_ang_acc(1);
  x_dot(11) = exerted_ang_acc(2);

  return x_dot;
}

casadi::MX DeltaController::calcStageCost(casadi::MX x, casadi::MX u)
{
  return dt_ * (dot(x - x_ref_, mtimes(eigenMatrixToCasadiDM(x_weight_mat_), x - x_ref_)) + dot(u, mtimes(eigenMatrixToCasadiDM(u_weight_mat_), u)));
}

casadi::MX DeltaController::calcTerminalCost(casadi::MX x)
{
  return dot(x - x_ref_, mtimes(eigenMatrixToCasadiDM(x_weight_mat_), x - x_ref_));
}


void DeltaController::controlCore()
{
  tf::Vector3 target_pos = navigator_->getTargetPos();
  tf::Vector3 target_vel = navigator_->getTargetVel();
  tf::Vector3 target_rpy = navigator_->getTargetRPY();
  tf::Vector3 target_omega = navigator_->getTargetOmega();

  /* set target x */
  x_ref_eigen_ <<
    target_pos.x(), target_pos.y(), target_pos.z(),
    target_vel.x(), target_vel.y(), target_vel.z(),
    target_rpy.x(), target_rpy.y(), target_rpy.z(),
    target_omega.x(), target_omega.y(), target_omega.z();
  x_ref_ = eigenVectorToCasadiDm(x_ref_eigen_);

  /* set initial x from previous solution and current state */
  Eigen::VectorXd initial_x_eigen = Eigen::VectorXd::Zero(nx_ + n_grid_ * (nx_ + nu_));
  initial_x_eigen = solution_;
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Vector3 euler = estimator_->getEuler(Frame::COG, estimate_mode_);
  tf::Vector3 omega = estimator_-> getAngularVel(Frame::COG, estimate_mode_);
  initial_x_eigen.head(nx_) <<
    pos.x(), pos.y(), pos.z(),
    vel.x(), vel.y(), vel.z(),
    euler.x(), euler.y(), euler.z(),
    omega.x(), omega.y(), omega.z();

  casadi::MX Xk = casadi::MX::sym("X0", nx_);
  casadi::MX J = 0;

  w_ = Xk;
  lbw_eigen_.resize(nx_ + n_grid_ * (nx_ + nu_));
  ubw_eigen_.resize(nx_ + n_grid_ * (nx_ + nu_));

  g_ = casadi::MX::zeros(n_grid_ * nx_);
  lbg_eigen_.resize(n_grid_ * nx_);
  ubg_eigen_.resize(n_grid_ * nx_);

  /* boundaries for initial x from current state*/
  lbw_eigen_.head(nx_) <<
    pos.x(), pos.y(), pos.z(),
    vel.x(), vel.y(), vel.z(),
    euler.x(), euler.y(), euler.z(),
    omega.x(), omega.y(), omega.z();
  ubw_eigen_.head(nx_) <<
    pos.x(), pos.y(), pos.z(),
    vel.x(), vel.y(), vel.z(),
    euler.x(), euler.y(), euler.z(),
    omega.x(), omega.y(), omega.z();

  /* prediction */
  for(int i = 0; i < n_grid_; i++)
    {
      casadi::MX Uk = casadi::MX::sym("U_" + std::to_string(i), nu_);
      J = J + calcStageCost(Xk, Uk);

      casadi::MX dXk = dynamics(Xk, Uk);
      casadi::MX Xk_next = Xk + dt_ * dXk;
      casadi::MX Xk1 = casadi::MX::sym("X_" + std::to_string(i + 1), nx_);

      /* boundaries */
      /* pos */
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 0) = target_pos.x() - 1.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 0) = target_pos.x() + 1.0;
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 1) = target_pos.y() - 1.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 1) = target_pos.y() + 1.0;
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 2) = target_pos.z() - 2.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 2) = target_pos.z() + 2.0;

      /* vel */
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 3) = target_vel.x() - 2.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 3) = target_vel.x() + 2.0;
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 4) = target_vel.y() - 2.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 4) = target_vel.y() + 2.0;
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 5) = target_vel.z() - 2.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 5) = target_vel.z() + 2.0;

      /* euler */
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 6) = target_rpy.x() - 1.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 6) = target_rpy.x() + 1.0;
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 7) = target_rpy.y() - 1.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 7) = target_rpy.y() + 1.0;
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 8) = target_rpy.z() - 1.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 8) = target_rpy.z() + 1.0;

      /* ang vel */
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 9) =  target_omega.x() - 3.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 9) =  target_omega.x() + 3.0;
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 10) = target_omega.y() - 3.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 10) = target_omega.y() + 3.0;
      lbw_eigen_(nx_ + i * (nx_ + nu_) + 11) = target_omega.z() - 3.0;
      ubw_eigen_(nx_ + i * (nx_ + nu_) + 11) = target_omega.z() + 3.0;

      w_ = vertcat(w_, Xk1);

      /* for u */
      Eigen::VectorXd static_thrust = robot_model_->getStaticThrust();
      for(int j = 0; j < nu_; j++)
        {
          lbw_eigen_(nx_ + i * (nx_ + nu_) + nx_ + j) = static_thrust(j) - 0.5;
          ubw_eigen_(nx_ + i * (nx_ + nu_) + nx_ + j) = static_thrust(j) + 0.5;
        }
      w_ = vertcat(w_, Uk);

      /* constraints */
      /* for x */
      for(int j = 0; j < nx_; j++)
        {
          g_(i * nx_ + j) = Xk_next(j) - Xk1(j);
          lbg_eigen_(i * nx_ + j) = -0.0;
          ubg_eigen_(i * nx_ + j) = 0.0;
        }

      Xk = Xk1;
    }
  J = J + calcTerminalCost(Xk);

  casadi::DM initial_x = eigenVectorToCasadiDm(initial_x_eigen);
  lbw_ = eigenVectorToCasadiDm(lbw_eigen_);
  ubw_ = eigenVectorToCasadiDm(ubw_eigen_);
  lbg_ = eigenVectorToCasadiDm(lbg_eigen_);
  ubg_ = eigenVectorToCasadiDm(ubg_eigen_);

  ROS_INFO_STREAM_ONCE("x_ref: " << x_ref_);
  ROS_INFO_STREAM_ONCE("\n");
  ROS_INFO_STREAM_ONCE("w: " << w_);
  ROS_INFO_STREAM_ONCE("\n");
  ROS_INFO_STREAM_ONCE("g: " << g_);
  ROS_INFO_STREAM_ONCE("\n");
  ROS_INFO_STREAM_ONCE("J: " << J);
  ROS_INFO_STREAM_ONCE("\n");

  casadi::MXDict nlp = {{"x", w_}, {"f", J}, {"g", g_}};
  casadi::Dict opt_dict = casadi::Dict();
  opt_dict["ipopt.max_iter"] = 50;
  opt_dict["ipopt.mu_min"] = 0.1;
  opt_dict["ipopt.warm_start_init_point"] = "yes";
  // opt_dict["ipopt.print_level"] = 0;
  // opt_dict["ipopt.sb"] = "yes";
  // opt_dict["print_time"] = 0;

  casadi::Function S = casadi::nlpsol("S", "ipopt", nlp, opt_dict);
  auto res = S(casadi::DMDict{{"x0", initial_x}, {"lbg", lbg_}, {"ubg", ubg_}, {"lbx", lbw_}, {"ubx", ubw_}, {"lam_x0", lam_x_}, {"lam_g0", lam_g_}});
  solution_ = casadiDmToEigenVector(res.at("x"));
  lam_x_ = res.at("lam_x");
  lam_g_ = res.at("lam_g");

  target_roll_ = solution_(nx_ + 6);
  target_pitch_ = solution_(nx_ + 7);
  candidate_yaw_term_ = yaw_p_gain_ * (target_rpy.z() - euler.z());

  for(int i = 0; i < nu_; i++)
    {
      target_base_thrust_.at(i) = solution_(2 * nx_ + i);
    }
  ROS_INFO_STREAM("f: " << res.at("f"));
  ROS_INFO_STREAM(solution_.segment(2 * nx_, 4).transpose());
}

void DeltaController::sendCmd()
{
  sendFourAxisCommand();
  sendTorqueAllocationMatrixInv();
}

void DeltaController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

void DeltaController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
      Eigen::MatrixXd torque_allocation_matrix_inv = aerial_robot_model::pseudoinverse(robot_model_->calcWrenchMatrixOnCoG()).rightCols(3);
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

void DeltaController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  ros::NodeHandle roll_pitch_nh_(nh_, "controller/roll_pitch");
  double p_gain, i_gain, d_gain;
  getParam<double>(roll_pitch_nh_, "p_gain", p_gain, 0.0);
  getParam<double>(roll_pitch_nh_, "i_gain", i_gain, 0.0);
  getParam<double>(roll_pitch_nh_, "d_gain", d_gain, 0.0);
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = p_gain * 1000;
  rpy_gain_msg.motors.at(0).roll_i = i_gain * 1000;
  rpy_gain_msg.motors.at(0).roll_d = d_gain * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = p_gain * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = i_gain * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = d_gain * 1000;

  ros::NodeHandle yaw_nh(nh_, "controller/yaw");
  getParam<double>(yaw_nh, "d_gain", d_gain, 0.0);
  rpy_gain_msg.motors.at(0).yaw_d = d_gain * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void DeltaController::visualizeFunc()
{
  ros::Rate loop_rate(10);
  while(ros::ok())
    {
      geometry_msgs::TransformStamped target_tf;
      target_tf.header.stamp = ros::Time::now();
      target_tf.header.frame_id = std::string("world");
      target_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("target"));
      target_tf.transform.translation.x = x_ref_eigen_(0);
      target_tf.transform.translation.y = x_ref_eigen_(1);
      target_tf.transform.translation.z = x_ref_eigen_(2);
      tf2::Quaternion q_target;
      q_target.setRPY(x_ref_eigen_(6), x_ref_eigen_(7), x_ref_eigen_(8));
      target_tf.transform.rotation.x = q_target.getX();
      target_tf.transform.rotation.y = q_target.getY();
      target_tf.transform.rotation.z = q_target.getZ();
      target_tf.transform.rotation.w = q_target.getW();
      br_.sendTransform(target_tf);

      for(int i = 0; i < n_grid_; i++)
        {
          geometry_msgs::TransformStamped predict_i_tf;
          predict_i_tf.header.stamp = ros::Time::now();
          predict_i_tf.header.frame_id = std::string("world");
          predict_i_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("predict" + std::to_string(i)));
          predict_i_tf.transform.translation.x = solution_(nx_ + i * (nx_ + nu_) + 0);
          predict_i_tf.transform.translation.y = solution_(nx_ + i * (nx_ + nu_) + 1);
          predict_i_tf.transform.translation.z = solution_(nx_ + i * (nx_ + nu_) + 2);
          tf2::Quaternion q;
          q.setRPY(solution_(nx_ + i * (nx_ + nu_) + 6), solution_(nx_ + i * (nx_ + nu_) + 7), solution_(nx_ + i * (nx_ + nu_) + 8));
          predict_i_tf.transform.rotation.x = q.getX();
          predict_i_tf.transform.rotation.y = q.getY();
          predict_i_tf.transform.rotation.z = q.getZ();
          predict_i_tf.transform.rotation.w = q.getW();
          br_.sendTransform(predict_i_tf);
        }
      loop_rate.sleep();
    }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::DeltaController, aerial_robot_control::ControlBase);
