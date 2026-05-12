// -*- mode: c++ -*-
#pragma once

#include <pinocchio/fwd.hpp>

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/multibody/fwd.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace aerial_robot_dynamics
{
class PinocchioRobotModel;
}

namespace aerial_robot_control
{

class FwddynMpcControlProblem
{
public:
  struct Parameters
  {
    int num_nodes = 50;
    int max_iter = 1;
    int max_init_iter = 100;
    double dt = 0.02;

    Eigen::Vector3d com_track_weight = Eigen::Vector3d::Constant(1e4);
    Eigen::VectorXd x_state_weight;
    double control_weight = 1e-3;
    double thrust_barrier_weight = 1e1;
    Eigen::VectorXd thrust_lb;
    Eigen::VectorXd thrust_ub;

    void print()
    {
      std::cout << "MPC Parameters:" << std::endl;
      std::cout << "  num_nodes: " << num_nodes << std::endl;
      std::cout << "  max_iter: " << max_iter << std::endl;
      std::cout << "  max_init_iter: " << max_init_iter << std::endl;
      std::cout << "  dt: " << dt << std::endl;
      std::cout << "  com_track_weight: " << com_track_weight.transpose() << std::endl;
      std::cout << "  control_weight: " << control_weight << std::endl;
      std::cout << "  thrust_barrier_weight: " << thrust_barrier_weight << std::endl;
      if (x_state_weight.size() > 0)
        std::cout << "  x_state_weight: " << x_state_weight.transpose() << std::endl;
      if (thrust_lb.size() > 0 && thrust_ub.size() > 0)
      {
        std::cout << "  thrust_lb: " << thrust_lb.transpose() << std::endl;
        std::cout << "  thrust_ub: " << thrust_ub.transpose() << std::endl;
      }
    }
  };

  FwddynMpcControlProblem() = default;

  void initialize(const std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel>& pinocchio_robot_model,
                  const Parameters& parameters);
  void reset();

  void buildMPCProblem(const Eigen::VectorXd& x0, const Eigen::Vector3d& com_target, const Eigen::VectorXd& x_ref);
  void setInitialState(const Eigen::VectorXd& x0);
  void slideHorizon(const Eigen::Vector3d& com_target, const Eigen::VectorXd& x_ref);
  bool solveMPC(int max_iter, bool verbose = false);

  bool isInitialized() const
  {
    return initialized_;
  }
  bool hasSolution() const
  {
    return !xs_.empty() && !us_.empty();
  }

  const Parameters& parameters() const
  {
    return parameters_;
  }
  const std::vector<Eigen::VectorXd>& xs() const
  {
    return xs_;
  }
  const std::vector<Eigen::VectorXd>& us() const
  {
    return us_;
  }

private:
  std::shared_ptr<crocoddyl::IntegratedActionModelEulerWithThrusts> createMPCNode(const Eigen::Vector3d& com_target,
                                                                                  const Eigen::VectorXd& x_ref);

  bool initialized_ = false;

  Parameters parameters_;

  std::shared_ptr<aerial_robot_dynamics::PinocchioRobotModel> pinocchio_robot_model_;
  std::shared_ptr<pinocchio::Model> pin_model_;

  std::shared_ptr<crocoddyl::StateMultibody> state_mb_;
  std::shared_ptr<crocoddyl::StateMultibodyWithThrusts> state_with_thrusts_;
  std::shared_ptr<crocoddyl::ActuationModelFloatingBaseThrusterRates> actuation_;

  std::shared_ptr<crocoddyl::ShootingProblem> shooting_problem_;
  std::shared_ptr<crocoddyl::SolverFDDP> solver_;

  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;

  std::vector<std::shared_ptr<crocoddyl::ResidualModelCoMPosition>> com_residuals_;
  std::vector<std::shared_ptr<crocoddyl::ResidualModelState>> state_residuals_;
};

}  // namespace aerial_robot_control
