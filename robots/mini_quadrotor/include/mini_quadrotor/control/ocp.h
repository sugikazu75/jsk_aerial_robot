#pragma once

#include <ros/package.h>

#include <aerial_robot_control/control/base/pose_linear_controller.h>
#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/initialization/OperatingPoints.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/PreComputation.h>
#include <ocs2_core/Types.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <spinal/FourAxisCommand.h>

namespace ocs2 {
  constexpr size_t STATE_DIM = 12;
  constexpr size_t INPUT_DIM = 4;

  class miniQuadrotorSystemDynamics final : public SystemDynamicsBase
  {
  public:
    miniQuadrotorSystemDynamics(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control);
    ~miniQuadrotorSystemDynamics() override = default;

    miniQuadrotorSystemDynamics* clone() const override { return new miniQuadrotorSystemDynamics(*this); }

    vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation&) override;
    VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override;

  private:
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;
  };


  class miniQuadrotorPreComputation : public PreComputation
  {
  public:
    miniQuadrotorPreComputation(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control);
    ~miniQuadrotorPreComputation() override = default;

    miniQuadrotorPreComputation(const miniQuadrotorPreComputation& rhs) = delete;
    miniQuadrotorPreComputation* clone() const override;

    void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;
    void requestFinal(RequestSet request, scalar_t t, const vector_t& x) override;

  private:
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;
  };


  class miniQuadrotorInterface final : public RobotInterface
  {
  public:
    miniQuadrotorInterface(const std::string& taskFile, const std::string& libraryFolder, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control);
    ~miniQuadrotorInterface() override = default;
    ddp::Settings& ddpSettings() { return ddpSettings_; }
    mpc::Settings& mpcSettings() { return mpcSettings_; }
    const OptimalControlProblem& getOptimalControlProblem() const override { return problem_; }
    std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }
    const RolloutBase& getRollout() const { return *rolloutPtr_; }

    const Initializer& getInitializer() const override { return *operatingPointPtr_; }

  private:
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;
    ddp::Settings ddpSettings_;
    mpc::Settings mpcSettings_;
    OptimalControlProblem problem_;
    std::shared_ptr<ReferenceManager> referenceManagerPtr_;
    std::unique_ptr<RolloutBase> rolloutPtr_;
    std::unique_ptr<Initializer> operatingPointPtr_;
  };
};

namespace aerial_robot_control{
  class ocp : public PoseLinearController
  {
  public:
    ocp();
    virtual ~ocp() = default;

    void initialize(ros::NodeHandle nh,
                    ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_du) override;

    virtual void reset() override;


  private:
    ros::Publisher flight_cmd_pub_; //for spinal
    spinal::FourAxisCommand four_axis_command_msg_;
    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control_;
    boost::shared_ptr<ocs2::miniQuadrotorInterface> mini_quadrotor_interface_;
    boost::shared_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;
    boost::shared_ptr<ocs2::MPC_MRT_Interface> mpc_interface_;
    bool first_run_;
    ocs2::TargetTrajectories target_trajectories_;
    ocs2::SystemObservation observation_;

    virtual void controlCore();
    virtual void sendCmd();

  };
};
