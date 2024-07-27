#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <crocoddyl/core/diff-action-base.hpp>
#include <crocoddyl/core/state-base.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/states/multibody.hpp>

#include <aerial_robot_control/control/base/pose_linear_controller.h>

namespace crocoddyl{
  class DifferentialActionModelFreeFwdDynamicsWithPropellers : public DifferentialActionModelFreeFwdDynamics{
  public:
    DifferentialActionModelFreeFwdDynamicsWithPropellers(const boost::shared_ptr<crocoddyl::StateMultibody>& state,
                                                         const boost::shared_ptr<crocoddyl::ActuationModelAbstract>& actuation,
                                                         const boost::shared_ptr<crocoddyl::CostModelSum>& costs,
                                                         const std::vector<std::string>& propeller_links);

    virtual void calc(const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& data,
                      const Eigen::Ref<const Eigen::VectorXd>& x,
                      const Eigen::Ref<const Eigen::VectorXd>& u) override;

    virtual void calcDiff(const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& data,
                          const Eigen::Ref<const Eigen::VectorXd>& x,
                          const Eigen::Ref<const Eigen::VectorXd>& u) override;


  protected:
    void updatePropellerPoses(const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& data,
                              const Eigen::Ref<const Eigen::VectorXd>& x);

  private:
    std::vector<std::string> propeller_links_;
    std::vector<pinocchio::SE3> propeller_poses_;
  };
};

namespace aerial_robot_control{
  class MpcController : public PoseLinearController
  {
  public:
    MpcController();
    virtual ~MpcController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;

    virtual void reset() override;

  private:
    bool first_run_;
    boost::shared_ptr<pinocchio::Model> model_;
    boost::shared_ptr<pinocchio::Data> data_;
    boost::shared_ptr<crocoddyl::StateMultibody> state_;

    virtual void controlCore() override;
    virtual void sendCmd() override;

    std::string getRobotModelXml(const std::string param, ros::NodeHandle nh = ros::NodeHandle());
  };
};
