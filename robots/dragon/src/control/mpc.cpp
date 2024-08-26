#include <dragon/control/mpc.h>

namespace crocoddyl{
  DifferentialActionModelFreeFwdDynamicsWithPropellers::DifferentialActionModelFreeFwdDynamicsWithPropellers(const boost::shared_ptr<crocoddyl::StateMultibody>& state,
                                                                                                             const boost::shared_ptr<crocoddyl::ActuationModelAbstract>& actuation,
                                                                                                             const boost::shared_ptr<crocoddyl::CostModelSum>& costs,
                                                                                                             const std::vector<std::string>& propeller_links):
    DifferentialActionModelFreeFwdDynamics(state, actuation, costs),
    propeller_links_(propeller_links)
  {
  }

  void DifferentialActionModelFreeFwdDynamicsWithPropellers::calc(const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& data,
                                                                  const Eigen::Ref<const Eigen::VectorXd>& x,
                                                                  const Eigen::Ref<const Eigen::VectorXd>& u)
  {
    updatePropellerPoses(data, x);
    DifferentialActionModelFreeFwdDynamics::calc(data, x, u);
  }

  void DifferentialActionModelFreeFwdDynamicsWithPropellers::calcDiff(const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& data,
                                                                      const Eigen::Ref<const Eigen::VectorXd>& x,
                                                                      const Eigen::Ref<const Eigen::VectorXd>& u)
  {
    updatePropellerPoses(data, x);
    DifferentialActionModelFreeFwdDynamics::calcDiff(data, x, u);
  }

  void DifferentialActionModelFreeFwdDynamicsWithPropellers::updatePropellerPoses(const boost::shared_ptr<crocoddyl::DifferentialActionDataAbstract>& data,
                                                                                  const Eigen::Ref<const Eigen::VectorXd>& x)
  {
    Data* d = static_cast<Data*>(data.get());

    pinocchio::forwardKinematics(get_pinocchio(), d->pinocchio, x);
    pinocchio::updateFramePlacements(get_pinocchio(), d->pinocchio);

    propeller_poses_.resize(propeller_links_.size());
    for (std::size_t i = 0; i < propeller_links_.size(); ++i)
      {
        propeller_poses_[i] = d->pinocchio.oMf[get_pinocchio().getFrameId(propeller_links_[i])];
      }
  }
}; // crocoddyl

namespace aerial_robot_control{
  MpcController::MpcController():
    PoseLinearController(),
    first_run_(true)
  {
  }

  void MpcController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                 boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                 double ctrl_loop_rate)
  {
    PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

    std::string robot_model_string = getRobotModelXml("robot_description");
    model_ = boost::make_shared<pinocchio::Model>();
    pinocchio::urdf::buildModelFromXML(robot_model_string, pinocchio::JointModelFreeFlyer(), *model_.get(), false);
    data_ = boost::make_shared<pinocchio::Data>(*model_);

    ROS_WARN_STREAM("[model][pinocchio] model name: " << model_->name);
    ROS_WARN_STREAM("[model][pinocchio] model.nq: " << model_->nq);
    ROS_WARN_STREAM("[model][pinocchio] model.nv: " << model_->nv);
    ROS_WARN_STREAM("[model][pinocchio] model.njoints: " << model_->njoints);

    state_ = boost::make_shared<crocoddyl::StateMultibody>(model_);
    actuation_ = boost::make_shared<crocoddyl::ActuationModelFloatingBase>(state_);
  }

  void MpcController::reset()
  {
    PoseLinearController::reset();
    first_run_ = true;
  }

  void MpcController::controlCore()
  {
    PoseLinearController::controlCore();
  }

  void MpcController::sendCmd()
  {
    PoseLinearController::sendCmd();
  }

  std::string MpcController::getRobotModelXml(const std::string param, ros::NodeHandle nh)
  {
    std::string xml_string;

    if(!nh.hasParam(param))
      {
        ROS_ERROR("Could not find parameter %s on parameter server with namespace '%s'", param.c_str(), nh.getNamespace().c_str());
        return xml_string;
      }
    nh.getParam(param, xml_string);
    return xml_string;
  }
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::MpcController, aerial_robot_control::ControlBase);
