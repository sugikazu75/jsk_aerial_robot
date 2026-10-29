// -*- mode: c++ -*-
#pragma once

#include <flying_humanoid/robot_model.h>
#include <flying_humanoid/solver_logger.h>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/multibody/actuations/floating-base-distributed-thrusters.hpp>
#include <crocoddyl/multibody/states/multibody-with-thrusts.hpp>

#include <Eigen/Core>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace flying_humanoid
{
// Gait phase of each knot, written to the CSV
enum GaitPhase
{
  DOUBLE_SUPPORT = 0,
  SWING_LEFT = 1,
  SWING_RIGHT = 2
};

struct WalkPlannerParams
{
  // gait
  std::string lfoot = "lfoot";  // foot frames: the sole is on the x-y plane
  std::string rfoot = "rfoot";
  double step_length = 0.06;  // [m] distance a foot moves in one step
  double step_height = 0.02;  // [m] height of the swing foot
  double time_step = 0.025;   // [s]
  int step_knots = 31;        // knots of one swing phase
  int support_knots = 8;      // knots of one double support phase
  double lateral_shift_scale = 0.5;  // how much the CoM moves toward the support foot (0: stays at the center)
  std::vector<double> initial_configuration;  // q: root position, root quaternion (x, y, z, w), joints. used when the
                                              // measured state is not available
  double m_f_rate = 0.0;                      // rotor torque per thrust [m]

  // solver
  int solver_type = 3;  // 0: FDDP, 1: box-FDDP, 3: HPIPM-SQP (see WalkPlanner.yaml)
  int max_iter = 1000;
  int num_threads = 4;

  // cost weights
  Eigen::VectorXd x_weights;  // 2 * nv
  double state_weight = 1.0;
  double control_weight = 0.01;
  Eigen::VectorXd thrust_reg_weight;  // rotor num
  double thrust_barrier_weight = 10.0;
  Eigen::Vector3d com_track_weight = Eigen::Vector3d(1e4, 1e5, 1e3);
  Eigen::VectorXd foot_track_weight = (Eigen::VectorXd(6) << 1e6, 1e6, 1e6, 1e3, 1e4, 1e4).finished();
  Eigen::VectorXd centroidal_momentum_weight = Eigen::VectorXd::Constant(6, 10.0);
  double contact_wrench_weight = 100.0;
  double contact_wrench_margin = 1.0;
  double state_bound_weight = 100.0;
  double state_bound_margin = 1.0;
  double minimum_contact_force = 5.0;  // [N]
  double mu = 0.7;
  Eigen::Vector2d foot_size = Eigen::Vector2d(0.08, 0.05);  // [m]
  Eigen::Vector2d contact_gains = Eigen::Vector2d(0.0, 50.0);
  double contact_jacobian_damping = 1e-6;
  double touchdown_velocity_weight = 1e4;  // the swing foot lands with zero velocity
  bool touchdown_constraint = true;        // ... exactly on its target (equality constraint, needs HPIPM-SQP)
  bool terminal_posture_constraint = true;  // end with the joint angles at the start
};

// Plans a walk of the given number of steps on flat ground by trajectory optimization.
// The thrusts are part of the state and the control input is their rate (thrust-rate mode), so the planned thrust
// changes smoothly.
class WalkPlanner
{
public:
  WalkPlanner(const std::string& robot_description, const WalkPlannerParams& params);

  // initial_configuration with the feet on the ground around the origin
  Eigen::VectorXd defaultConfiguration();
  // configuration from the measured pose of the baselink and the measured joint positions
  Eigen::VectorXd measuredConfiguration(const pinocchio::SE3& world_M_baselink,
                                        const std::map<std::string, double>& joint_positions,
                                        const std::string& baselink);

  // plan a walk starting from q_start (with zero velocity)
  // returns false when the solver did not converge (the result is still available)
  bool plan(int num_steps, const Eigen::VectorXd& q_start);
  bool saveCsv(const std::string& filename) const;

  // for the analysis of the convergence
  bool saveSolverLog(const std::string& filename) const;  // solver quantities of each iteration
  bool saveNodeDiagnosis(const std::string& filename);    // cost terms, constraint violations and gaps of each node
  void checkDerivatives();  // compare the analytic derivatives with finite differences at some nodes

private:
  typedef std::vector<std::pair<pinocchio::FrameIndex, pinocchio::SE3>> FootTasks;

  void createProblem(int num_steps);
  void addStepModels(const std::vector<pinocchio::FrameIndex>& support_feet, const FootTasks& swing_target,
                     FootTasks& feet, Eigen::Vector3d& com_ref);
  void addKnot(const Eigen::Vector3d& com_ref, const FootTasks& support_feet, const FootTasks& swing_feet,
               double left_foot_weight = 0.5, bool touchdown = false, bool terminal = false);

  WalkPlannerParams params_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::vector<RotorInfo> rotors_;
  std::vector<crocoddyl::DistributedThruster> thrusters_;
  std::shared_ptr<crocoddyl::StateMultibody> state_;
  std::shared_ptr<crocoddyl::StateMultibodyWithThrusts> state_with_thrusts_;
  pinocchio::FrameIndex lfoot_id_, rfoot_id_;

  Eigen::VectorXd q0_;       // configuration at the start
  Eigen::Matrix3d heading_;  // yaw of the root at the start
  std::vector<std::shared_ptr<crocoddyl::ActionModelAbstract>> knots_;
  std::vector<int> gait_phases_;
  std::shared_ptr<crocoddyl::ShootingProblem> problem_;
  std::vector<Eigen::VectorXd> xs_;
  std::vector<Eigen::VectorXd> us_;
  std::vector<Eigen::VectorXd> fs_;
  std::shared_ptr<SolverLogger> logger_;
};
}  // namespace flying_humanoid
