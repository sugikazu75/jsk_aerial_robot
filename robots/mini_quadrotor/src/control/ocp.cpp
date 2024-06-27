#include <mini_quadrotor/control/ocp.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

namespace ocs2
{
  miniQuadrotorSystemDynamics::miniQuadrotorSystemDynamics(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control):
    robot_model_for_control_(robot_model_for_control)
  {
  }

  vector_t miniQuadrotorSystemDynamics::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&)
  {
    int dqQ_index = 6;
    int qw_index = 9;

    Eigen::Matrix<scalar_t, 3, 1> eulerAngle = state.segment<3>(3);
    Eigen::Matrix<scalar_t, 3, 3> T = getMappingFromLocalAngularVelocityToEulerAnglesXyzDerivative<scalar_t>(eulerAngle);

    // positions
    scalar_t qxp = state(0);  // x
    scalar_t qyp = state(1);  // y
    scalar_t qzp = state(2);  // z

    // quaternion
    // scalar_t qqx = state(3);
    // scalar_t qqy = state(4);
    // scalar_t qqz = state(5);
    // scalar_t qqw = state(6);

    // euler
    scalar_t qxa = state(3);
    scalar_t qya = state(4);
    scalar_t qza = state(5);

    // positions derivatives
    scalar_t dqxp = state(dqQ_index + 0);  // x
    scalar_t dqyp = state(dqQ_index + 1);  // y
    scalar_t dqzp = state(dqQ_index + 2);  // z

    // angular velocity xyz
    scalar_t qxw = state(qw_index + 0);
    scalar_t qyw = state(qw_index + 1);
    scalar_t qzw = state(qw_index + 2);

    // Euler angle derivatives
    Eigen::Matrix<scalar_t, 3, 1> eulerAngleDerivatives = T * state.segment<3>(qw_index);

    // derivative
    vector_t stateDerivative(STATE_DIM);

    // Eigen::Quaternion quaternion(state(6), state(3), state(4), state(5)); // Eigen::Quaternion is (w, x, y, z). w_Q_cog
    // Eigen::Matrix R = quaternion.toRotationMatrix(); // w_R_cog
    tf::Matrix3x3 r; r.setRPY(qxa, qya, qza);
    Eigen::Matrix3d R; tf::matrixTFToEigen(r, R);

    vector_t thrust(INPUT_DIM); thrust << input(0), input(1), input(2), input(3);
    // Eigen::MatrixXd q_mat = robot_model_for_control_->calcWrenchMatrixOnCoG();
    Eigen::MatrixXd q_mat = robot_model_for_control_->getQ();

    double mass = robot_model_for_control_->getMass();
    Eigen::VectorXd f_cog = (q_mat * thrust).topRows(3);
    Eigen::VectorXd ddqp = R * f_cog / mass - robot_model_for_control_->getGravity3d();

    Eigen::Matrix3d inertia = robot_model_for_control_->getInertia<Eigen::Matrix3d>();
    Eigen::VectorXd tau_cog = (q_mat * thrust).bottomRows(3);
    Eigen::Vector3d omega; omega << state(qw_index + 0), state(qw_index + 1), state(qw_index + 2);
    Eigen::VectorXd gyro_moment = - omega.cross(inertia * omega);
    Eigen::VectorXd domega = inertia.inverse() * (gyro_moment + tau_cog);

    stateDerivative(0) = dqxp;
    stateDerivative(1) = dqyp;
    stateDerivative(2) = dqzp;
    stateDerivative(3) = eulerAngleDerivatives(0);
    stateDerivative(4) = eulerAngleDerivatives(1);
    stateDerivative(5) = eulerAngleDerivatives(2);
    // stateDerivative(3) = 1.0 / 2.0 * (qqx  * 0    + qqy * dqps - qqz * dqth + qqw * dqph);
    // stateDerivative(4) = 1.0 / 2.0 * (-qqx * dqps + qqy * 0    + qqz * dqph + qqw * dqth);
    // stateDerivative(5) = 1.0 / 2.0 * (qqx  * dqth - qqy * dqph + qqz * 0    + qqw * dqps);
    // stateDerivative(6) = 1.0 / 2.0 * (-qqx * dqph - qqy * dqth - qqz * dqps + qqw * 0);
    stateDerivative(dqQ_index + 0) = ddqp(0);
    stateDerivative(dqQ_index + 1) = ddqp(1);
    stateDerivative(dqQ_index + 2) = ddqp(2);
    stateDerivative(qw_index + 0) = domega(0);
    stateDerivative(qw_index + 1) = domega(1);
    stateDerivative(qw_index + 2) = domega(2);

    return stateDerivative;
  }

  VectorFunctionLinearApproximation miniQuadrotorSystemDynamics::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                                                     const PreComputation& preComp)
  {
    VectorFunctionLinearApproximation dynamics;
    dynamics.f = computeFlowMap(t, x, u, preComp);

    matrix_t& A = dynamics.dfdx; A.setZero(STATE_DIM, STATE_DIM);
    matrix_t& B = dynamics.dfdu; B.setZero(STATE_DIM, INPUT_DIM);
    const  scalar_t eps = 1e-6;
    for(int i = 0; i < STATE_DIM; i++)
      {
        vector_t state_eps = x;
        state_eps(i) += eps;
        vector_t f_eps = computeFlowMap(t, state_eps, u, preComp);
        A.col(i) = (f_eps - dynamics.f) / eps;
      }
    for(int i = 0; i < INPUT_DIM; i++)
      {
        vector_t input_eps = u;
        input_eps(i) += eps;
        vector_t f_eps = computeFlowMap(t, x, input_eps, preComp);
        B.col(i) = (f_eps - dynamics.f) / eps;
      }

    return dynamics;
  }

  miniQuadrotorPreComputation::miniQuadrotorPreComputation(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control):
    robot_model_for_control_(robot_model_for_control)
  {
  }

  miniQuadrotorPreComputation* miniQuadrotorPreComputation::clone() const
  {
    return new miniQuadrotorPreComputation(robot_model_for_control_);
  }

  void miniQuadrotorPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u)
  {
    return;
  }
  void miniQuadrotorPreComputation::requestFinal(RequestSet request, scalar_t t, const vector_t& x)
  {
    return;
  }


  miniQuadrotorInterface::miniQuadrotorInterface(const std::string& taskFile, const std::string& libraryFolder, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control):
    robot_model_for_control_(robot_model_for_control)
  {
    // check that task file exists
    boost::filesystem::path taskFilePath(taskFile);
    if (boost::filesystem::exists(taskFilePath)) {
      std::cerr << "[miniQuadrotorInterface] Loading task file: " << taskFilePath << std::endl;
    } else {
      throw std::invalid_argument("[miniQuadrotorInterface] Task file not found: " + taskFilePath.string());
    }

    // create library folder if it does not exist
    boost::filesystem::path libraryFolderPath(libraryFolder);
    boost::filesystem::create_directories(libraryFolderPath);
    std::cerr << "[miniQuadrotorInterface] Generated library path: " << libraryFolderPath << std::endl;

    // Solver settings
    ddpSettings_ = ddp::loadSettings(taskFile, "ddp");
    mpcSettings_ = mpc::loadSettings(taskFile, "mpc");

    /*
     * ReferenceManager & SolverSynchronizedModule
     */
    referenceManagerPtr_.reset(new ReferenceManager);

    // Cost
    matrix_t Q(STATE_DIM, STATE_DIM);
    matrix_t R(INPUT_DIM, INPUT_DIM);
    matrix_t Qf(STATE_DIM, STATE_DIM);
    loadData::loadEigenMatrix(taskFile, "Q", Q);
    loadData::loadEigenMatrix(taskFile, "R", R);
    loadData::loadEigenMatrix(taskFile, "Q_final", Qf);
    std::cerr << "Q:  \n" << Q << "\n";
    std::cerr << "R:  \n" << R << "\n";
    std::cerr << "Q_final:\n" << Qf << "\n";

    problem_.costPtr->add("cost", std::make_unique<QuadraticStateInputCost>(Q, R));
    problem_.finalCostPtr->add("finalCost", std::make_unique<QuadraticStateCost>(Qf));

    // Dynamics
    problem_.dynamicsPtr.reset(new miniQuadrotorSystemDynamics(robot_model_for_control_));

    // PreComputation
    problem_.preComputationPtr.reset(new miniQuadrotorPreComputation(robot_model_for_control_));

    // Rollout
    auto rolloutSettings = rollout::loadSettings(taskFile, "rollout");
    rolloutPtr_.reset(new TimeTriggeredRollout(*problem_.dynamicsPtr, rolloutSettings));

    // Constrains
    auto getPenalty = [&]()
                      {
                        // one can use either augmented::SlacknessSquaredHingePenalty or augmented::ModifiedRelaxedBarrierPenalty
                        using penalty_type = augmented::SlacknessSquaredHingePenalty;
                        penalty_type::Config boundsConfig;
                        loadData::loadPenaltyConfig(taskFile, "bounds_penalty_config", boundsConfig);
                        return penalty_type::create(boundsConfig);
                      };
    auto getConstraint = [&]()
                         {
                           // C * x + D * u + e >= 0
                           double thrust_lower_limit = robot_model_for_control_->getThrustLowerLimit();
                           double thrust_upper_limit = robot_model_for_control_->getThrustUpperLimit();
                           constexpr size_t numIneqConstraint = INPUT_DIM * 2;
                           vector_t e = vector_t::Ones(numIneqConstraint);
                           e.head(INPUT_DIM) = -thrust_lower_limit * e.head(INPUT_DIM);
                           e.tail(INPUT_DIM) = thrust_upper_limit * e.tail(INPUT_DIM);
                           matrix_t D = matrix_t::Identity(numIneqConstraint, INPUT_DIM);
                           D.topRows(INPUT_DIM) = 1.0 * matrix_t::Identity(INPUT_DIM, INPUT_DIM);
                           D.bottomRows(INPUT_DIM) = -1.0 *  matrix_t::Identity(INPUT_DIM, INPUT_DIM);
                           const matrix_t C = matrix_t::Zero(numIneqConstraint, STATE_DIM);
                           std::cout << "e:\n" << e.transpose() << std::endl;
                           std::cout << "D:\n" << D << std::endl;
                           std::cout << "C:\n" << C << std::endl;
                           return std::make_unique<LinearStateInputConstraint>(e, C, D);
                         };
    problem_.inequalityLagrangianPtr->add("InputLimits", create(getConstraint(), getPenalty()));

    // Initialization
    vector_t initialState = vector_t::Zero(STATE_DIM);
    vector_t initialInput = vector_t::Zero(INPUT_DIM);
    operatingPointPtr_.reset(new OperatingPoints(initialState, initialInput));
  }
}; // ocs2


ocp::ocp():
  PoseLinearController(),
  first_run_(true)
{
}

void ocp::initialize(ros::NodeHandle nh,
                     ros::NodeHandle nhp,
                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                     double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  robot_model_for_control_ = boost::make_shared<aerial_robot_model::RobotModel>();
  robot_model_for_control_->updateRobotModel();

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);

  const std::string taskFile = ros::package::getPath("mini_quadrotor") + "/config/mpc/task.info";
  const std::string libFolder = ros::package::getPath("mini_quadrotor") + "/auto_generated";
  mini_quadrotor_interface_ = boost::make_shared<ocs2::miniQuadrotorInterface>(taskFile, libFolder, robot_model_for_control_);

  mpc_ = boost::make_shared<ocs2::GaussNewtonDDP_MPC>(mini_quadrotor_interface_->mpcSettings(),
                                                      mini_quadrotor_interface_->ddpSettings(),
                                                      mini_quadrotor_interface_->getRollout(),
                                                      mini_quadrotor_interface_->getOptimalControlProblem(),
                                                      mini_quadrotor_interface_->getInitializer());
  mpc_->getSolverPtr()->setReferenceManager(mini_quadrotor_interface_->getReferenceManagerPtr());

  mpc_interface_ = boost::make_shared<ocs2::MPC_MRT_Interface>(*mpc_);
}

void ocp::reset()
{
  PoseLinearController::reset();
  first_run_ = true;
  target_trajectories_.clear();
  mpc_interface_->resetMpcNode(target_trajectories_);
}

void ocp::controlCore()
{
  PoseLinearController::controlCore();

  robot_model_for_control_->updateRobotModel();
  robot_model_for_control_->calcWrenchMatrixOnCoG();

  double current_time = ros::Time::now().toSec();

  tf::Vector3 target_pos = navigator_->getTargetPos();
  tf::Vector3 target_vel = navigator_->getTargetVel();
  tf::Vector3 target_rpy = navigator_->getTargetRPY();
  tf::Quaternion target_quaternion; target_quaternion.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
  tf::Vector3 target_omega = navigator_->getTargetOmega();

  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  tf::Vector3 vel = estimator_->getVel(Frame::COG, estimate_mode_);
  tf::Matrix3x3 R = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 euler = estimator_->getEuler(Frame::COG, estimate_mode_);
  tf::Quaternion quaternion; R.getRotation(quaternion);
  tf::Vector3 omega = estimator_->getAngularVel(Frame::COG, estimate_mode_);

  ocs2::vector_t mpc_state(ocs2::STATE_DIM);
  mpc_state <<
    pos.x(), pos.y(), pos.z(),
    // quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w(),
    euler.x(), euler.y(), euler.z(),
    vel.x(), vel.y(), vel.z(),
    omega.x(), omega.y(), omega.z();

  ocs2::vector_t mpc_target_state(ocs2::STATE_DIM);
  mpc_target_state <<
    target_pos.x(), target_pos.y(), target_pos.z(),
    // target_quaternion.x(), target_quaternion.y(), target_quaternion.z(), target_quaternion.w(),
    target_rpy.x(), target_rpy.y(), target_rpy.z(),
    target_vel.x(), target_vel.y(), target_vel.z(),
    target_omega.x(), target_omega.y(), target_omega.z();

  Eigen::VectorXd mpc_target_input = aerial_robot_model::pseudoinverse(robot_model_for_control_->calcWrenchMatrixOnCoG()) * robot_model_for_control_->getMass() * robot_model_for_control_->getGravity();

  target_trajectories_ = ocs2::TargetTrajectories({current_time}, {mpc_target_state}, {ocs2::vector_t::Zero(ocs2::INPUT_DIM)});

  mini_quadrotor_interface_->getReferenceManagerPtr()->setTargetTrajectories(std::move(target_trajectories_));

  if(first_run_)
    {
      observation_.time = current_time;
      observation_.state = mpc_state;
      observation_.input.setZero(ocs2::INPUT_DIM);
      mpc_interface_->setCurrentObservation(observation_);

      first_run_ = false;
    }

  mpc_interface_->advanceMpc();

  if (mpc_interface_->initialPolicyReceived())
    {
      size_t mode;
      ocs2::vector_t optimalState, optimalInput;

      mpc_interface_->updatePolicy();
      mpc_interface_->evaluatePolicy(current_time, mpc_state, optimalState, optimalInput, mode);

      // // use optimal state for the next observation:
      observation_.time = current_time;
      observation_.state = mpc_state;
      observation_.input = optimalInput;
      mpc_interface_->setCurrentObservation(observation_);

      std::vector<float> target_base_thrust(4);
      for(int i = 0; i < motor_num_; i++)
        {
          target_base_thrust.at(i) = optimalInput(i);
        }
      four_axis_command_msg_.base_thrust = target_base_thrust;
    }
}

void ocp::sendCmd()
{
  PoseLinearController::sendCmd();

  flight_cmd_pub_.publish(four_axis_command_msg_);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::ocp, aerial_robot_control::ControlBase);
