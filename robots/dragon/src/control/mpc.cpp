#include <dragon/control/mpc.h>

using namespace aerial_robot_control;

namespace ocs2
{
  dragonSystemDyamicsAD::dragonSystemDyamicsAD(const PinocchioInterface& pinocchioInterface,
                                               const CentroidalModelInfo& info,
                                               const std::string& modelName):
    pinocchioCentroidalDynamicsAd_(pinocchioInterface, info, modelName) {}

  vector_t dragonSystemDyamicsAD::computeFlowMap(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) {
    return pinocchioCentroidalDynamicsAd_.getValue(time, state, input);
  }

  VectorFunctionLinearApproximation dragonSystemDyamicsAD::linearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                               const PreComputation& preComp) {
    return pinocchioCentroidalDynamicsAd_.getLinearApproximation(time, state, input);
  }
};


mpc::mpc():
  ControlBase()
{
}

void mpc::initialize(ros::NodeHandle nh,
                     ros::NodeHandle nhp,
                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                     double ctrl_loop_rate)
{
  ControlBase::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
}

void mpc::reset()
{
  ControlBase::reset();
}

bool mpc::update()
{
  if(!ControlBase::update()) return false;

  controlCore();
  sendCmd();

  return true;
}

void mpc::controlCore()
{
  ROS_INFO_STREAM("mpc controlcore");
}

void::mpc::sendCmd()
{
  ROS_INFO_STREAM("mpc send command");
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc, aerial_robot_control::ControlBase);
