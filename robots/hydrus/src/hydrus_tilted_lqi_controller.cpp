#include <hydrus/hydrus_tilted_lqi_controller.h>

using namespace aerial_robot_control;

HydrusTiltedLQIController::HydrusTiltedLQIController():
  UnderActuatedTiltedLQIController()
{
}

void HydrusTiltedLQIController::initialize(ros_compat::NodeHandle nh,
                                     ros_compat::NodeHandle nhp,
                                     ros_compat::SharedPtr<aerial_robot_model::RobotModel> robot_model,
                                     ros_compat::SharedPtr<aerial_robot_estimation::StateEstimator> estimator,
                                     ros_compat::SharedPtr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedTiltedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
}

bool HydrusTiltedLQIController::checkRobotModel()
{
  if(!robot_model_->initialized())
    {
      ROS_COMPAT_DEBUG("LQI gain generator: robot model is not initiliazed");
      return false;
    }

  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_COMPAT_ERROR("LQI gain generator: invalid pose, stability is invalid");

      return false;
    }
  return true;
}

/* plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQIController, aerial_robot_control::ControlBase);
