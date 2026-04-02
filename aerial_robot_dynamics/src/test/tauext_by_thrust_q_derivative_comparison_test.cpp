#include <aerial_robot_dynamics/robot_model_test.h>
#include <chrono>

using namespace aerial_robot_dynamics;

bool PinocchioRobotModelTest::computeTauExtByThrustQDerivativeComparisonTest(bool verbose)
{
  Eigen::VectorXd q = pinocchio::randomConfiguration(*(robot_model_->getModel()));
  Eigen::VectorXd thrust = Eigen::VectorXd::Ones(robot_model_->getRotorNum());

  addNoise(thrust, 0.1);

  int nv = robot_model_->getModel()->nv;
  Eigen::MatrixXd tauext_by_thrust_q_derivative_hessian = Eigen::MatrixXd::Zero(nv, nv);
  Eigen::MatrixXd tauext_by_thrust_q_derivative_rnea = Eigen::MatrixXd::Zero(nv, nv);

  // hessian method
  {
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q =
        robot_model_->computeTauExtByThrustDerivativeQDerivatives(q);  // compute analytical derivatives
    for (int i = 0; i < nv; i++)
    {
      tauext_by_thrust_q_derivative_hessian.col(i) = tauext_partial_thrust_partial_q.at(i) * thrust;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "TauExt by Thrust Q Derivative Hessian time: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
              << std::endl;

    if (verbose)
    {
      std::cout << "tauext_by_thrust_q_derivative_hessian" << std::endl;
      std::cout << tauext_by_thrust_q_derivative_hessian << std::endl;
    }
  }

  // rnea method
  {
    auto start = std::chrono::high_resolution_clock::now();
    tauext_by_thrust_q_derivative_rnea = robot_model_->computeTauExtByThrustQDerivative(q, thrust);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "TauExt by Thrust Q Derivative RNEA time: "
              << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms"
              << std::endl;

    if (verbose)
    {
      std::cout << "tauext_by_thrust_q_derivative_rnea" << std::endl;
      std::cout << tauext_by_thrust_q_derivative_rnea << std::endl;
    }
  }

  return tauext_by_thrust_q_derivative_hessian.isApprox(tauext_by_thrust_q_derivative_rnea, 1e-4);
}
