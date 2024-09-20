#pragma once

#include <pinocchio/fwd.hpp>
#include <array>
#include <aerial_robot_control/control/base/base.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamicsAD.h>
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
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>

namespace ocs2
{
  class dragonSystemDyamicsAD final : public SystemDynamicsBase
  {
    dragonSystemDyamicsAD(const PinocchioInterface& pinocchioInterface,
                          const CentroidalModelInfo& info,
                          const std::string& modelName);

    ~dragonSystemDyamicsAD() override = default;
    dragonSystemDyamicsAD* clone() const override { return new dragonSystemDyamicsAD(*this); }

    vector_t computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) override;
    VectorFunctionLinearApproximation linearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                          const PreComputation& preComp) override;
  private:
    dragonSystemDyamicsAD(const dragonSystemDyamicsAD& rhs) = default;

    PinocchioCentroidalDynamicsAD pinocchioCentroidalDynamicsAd_;
  };

  class dragonInterface final : public RobotInterface
  {
  public:
    dragonInterface(const std::string& taskFile, const std::string& urdfFile);
    ~dragonInterface() override = default;

    ddp::Settings& ddpSettings() { return ddpSettings_; }
    mpc::Settings& mpcSettings() { return mpcSettings_; }
    const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }
    std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }
    const RolloutBase& getRollout() const { return *rolloutPtr_; }
    const Initializer& getInitializer() const override { return *operatingPointPtr_; }

    PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }

  private:
    ddp::Settings ddpSettings_;
    mpc::Settings mpcSettings_;
    std::unique_ptr<OptimalControlProblem> problemPtr_;
    std::shared_ptr<ReferenceManager> referenceManagerPtr_;
    std::unique_ptr<RolloutBase> rolloutPtr_;
    std::unique_ptr<Initializer> operatingPointPtr_;

    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  };
};

namespace aerial_robot_control
{
  class mpc : public ControlBase
  {
  public:
    mpc();
    virtual ~mpc() = default;

    void initialize(ros::NodeHandle nh,
                    ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_control,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_du) override;

    virtual bool update() override;
    virtual void reset() override;

  private:
    virtual void controlCore();
    virtual void sendCmd();
  };
};
