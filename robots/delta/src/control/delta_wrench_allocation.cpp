#include <delta/control/delta_wrench_allocation.h>
#include <nlopt.hpp>

using namespace aerial_robot_control;

double nonlinearWrenchAllocationMinObjective(const std::vector<double>& x, std::vector<double>& grad, void* ptr)
{
  // 0 ~ motor_num : lambda
  // motor_num ~ 2 * motor_num : gimbal roll (phi)

  DeltaController* controller = reinterpret_cast<DeltaController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  controller->setNLOptIterations(controller->getNLOptIterations() + 1);

  double ret = 0;
  int motor_num = robot_model_for_control->getRotorNum();

  /* compute result */
  // f = sum(lambda_i * lambda_i)
  for (int i = 0; i < motor_num; i++)
  {
    ret += x.at(i) * x.at(i);
  }

  if (grad.empty())
    return ret;

  /* make gradient */
  // df/dlambda_i = 2 * lambda_i
  // df/dphi_i = 0
  for (int i = 0; i < motor_num; i++)
  {
    grad.at(i) = 2 * x.at(i);
    grad.at(i + motor_num) = 0;
  }
  return ret;
}

void nonlinearWrenchAllocationEqConstraints(unsigned m, double* result, unsigned n, const double* x, double* grad,
                                            void* ptr)
{
  // 0 ~ 6 : Q(phi) * lambda - target_wrench_cog(phi) = 0
  // the rotor direction u, the rotor position p and the inertia I are calculated by the kinematics model,
  // since all of them (p and I via the CoG shift) depend on the gimbal angles

  DeltaController* controller = reinterpret_cast<DeltaController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  int motor_num = robot_model_for_control->getRotorNum();

  /* reflect the gimbal angles of the decision variables to the model. link joints are kept */
  const std::vector<int>& gimbal_joint_indices = robot_model_for_control->getGimbalJointIndices();
  KDL::JntArray joint_positions = robot_model_for_control->getJointPositions();
  for (int i = 0; i < motor_num; i++)
    joint_positions(gimbal_joint_indices.at(i)) = x[i + motor_num];
  robot_model_for_control->updateRobotModel(joint_positions);

  std::vector<Eigen::Vector3d> rotors_origin_from_cog =
      robot_model_for_control->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> rotors_normal_from_cog =
      robot_model_for_control->getRotorsNormalFromCog<Eigen::Vector3d>();

  Eigen::VectorXd target_acc_cog = controller->getTargetAccCog();
  double mass = robot_model_for_control->getMass();
  Eigen::Matrix3d inertia = robot_model_for_control->getInertia<Eigen::Matrix3d>();
  double m_f_rate = robot_model_for_control->getMFRate();
  std::map<int, int> rotor_direction = robot_model_for_control->getRotorDirection();

  Eigen::VectorXd target_wrench_cog = target_acc_cog;
  target_wrench_cog.head(3) = mass * target_wrench_cog.head(3);
  target_wrench_cog.tail(3) = inertia * target_wrench_cog.tail(3);

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(motor_num);
  for (int i = 0; i < motor_num; i++)
    lambda(i) = x[i];

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, motor_num);
  for (int i = 0; i < motor_num; i++)
  {
    Q.block(0, i, 3, 1) = rotors_normal_from_cog.at(i);
    Q.block(3, i, 3, 1) = (aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                           m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3)) *
                          rotors_normal_from_cog.at(i);
  }

  Eigen::VectorXd wrench_diff = Q * lambda - target_wrench_cog;
  for (int i = 0; i < m; i++)
    result[i] = wrench_diff(i);

  if (grad == NULL)
    return;

  /* make gradient */
  // thrust part. dwrench_dlambda = Q */
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < motor_num; j++)
    {
      grad[i * n + j] = Q(i, j);
    }
  }

  /* gimbal part */
  // df/dphi_j = sum_i(lambda_i * du_i/dphi_j)
  // dtau/dphi_j = sum_i(lambda_i * (dp_i/dphi_j x u_i + p_i x du_i/dphi_j + sigma_i * du_i/dphi_j)) - dI/dphi_j * alpha
  // every rotor contributes to the derivative w.r.t. each gimbal, because the gimbal motion shifts the CoG
  robot_model_for_control->calcCoGMomentumJacobian();  // should be processed first
  robot_model_for_control->calcInertiaJacobian();
  robot_model_for_control->calcBasicKinematicsJacobian();  // need cog jacobian

  const std::vector<int>& gimbal_jacobian_cols = robot_model_for_control->getGimbalJacobianCols();
  const std::vector<Eigen::MatrixXd>& u_jacobians = robot_model_for_control->getUJacobians();
  const std::vector<Eigen::MatrixXd>& p_jacobians = robot_model_for_control->getPJacobians();
  const std::vector<Eigen::Matrix3d>& inertia_jacobian = robot_model_for_control->getInertiaJacobian();

  Eigen::MatrixXd df_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  Eigen::MatrixXd dtau_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  for (int j = 0; j < motor_num; j++)
  {
    const int col = gimbal_jacobian_cols.at(j);
    for (int i = 0; i < motor_num; i++)
    {
      const Eigen::Vector3d du = u_jacobians.at(i).col(col);
      const Eigen::Vector3d dp = p_jacobians.at(i).col(col);

      df_dphi.col(j) += x[i] * du;
      dtau_dphi.col(j) += x[i] * (dp.cross(rotors_normal_from_cog.at(i)) + rotors_origin_from_cog.at(i).cross(du) +
                                  m_f_rate * rotor_direction.at(i + 1) * du);
    }
    dtau_dphi.col(j) -= inertia_jacobian.at(col) * target_acc_cog.tail(3);
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < motor_num; j++)
    {
      grad[i * n + (j + motor_num)] = df_dphi(i, j);
      grad[(i + 3) * n + (j + motor_num)] = dtau_dphi(i, j);
    }
  }
}
