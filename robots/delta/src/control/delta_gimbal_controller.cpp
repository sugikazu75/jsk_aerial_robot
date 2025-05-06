#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void RollingController::gimbalPlanner()
{
  auto gimbal_planning_flag = rolling_robot_model_->getGimbalPlanningFlag();
  auto current_gimbal_planning_angle = rolling_robot_model_->getCurrentGimbalPlanningAngle();
  auto final_gimbal_planning_angle = rolling_robot_model_->getFinalGimbalPlanningAngle();
  auto current_gimbal_angles = rolling_robot_model_->getCurrentGimbalAngles();

  for(int i = 0; i < motor_num_; i++)
    {
      if(!gimbal_planning_flag.at(i)) continue; // planning flag check

      // convergence check
      if(fabs(current_gimbal_angles.at(i) - final_gimbal_planning_angle.at(i)) < gimbal_planning_converged_thresh_)
        {
          ROS_INFO_STREAM("[control] finish planning for gimbal" << i + 1 << " current: " << current_gimbal_angles.at(i) << " target: " << final_gimbal_planning_angle.at(i));
          rolling_robot_model_->setGimbalPlanningFlag(i, 0);
          continue;
        }

      double current_target_angle = current_gimbal_planning_angle.at(i) + sign(final_gimbal_planning_angle.at(i) - current_gimbal_planning_angle.at(i)) * gimbal_velocity_ * ctrl_loop_du_;
      rolling_robot_model_->setCurrentGimbalPlanningAngle(i, current_target_angle);
      target_gimbal_angles_.at(i) = current_target_angle;
      ROS_INFO_STREAM_THROTTLE(0.1, "[control] gimbal" << i + 1 << "  current target: " << current_target_angle << " current angle: " << current_gimbal_angles.at(i) << " final target: " << final_gimbal_planning_angle.at(i));
    }
}

void RollingController::gimbalPlanningCallback(const delta::GimbalPlanningPtr & msg)
{
  if(msg->index.size() > 1)
    {
      ROS_ERROR_STREAM("[control] plan " << msg->index.size() << " gimbals is impossible");
      return;
    }
  if(msg->index[0] < 0 || motor_num_ - 1 < msg->index[0])
    {
      ROS_ERROR_STREAM("[control] given gimbal index " << msg->index[0] << " is out of range");
      return;
    }

  auto gimbal_planning_flag = rolling_robot_model_->getGimbalPlanningFlag();
  int num_of_planned_gimbals = std::accumulate(gimbal_planning_flag.begin(), gimbal_planning_flag.end(), 0);
  auto current_gimbal_angles = rolling_robot_model_->getCurrentGimbalAngles();

  if(num_of_planned_gimbals == 1)
    {
      if(gimbal_planning_flag.at(msg->index[0]) == 0)
        {
          ROS_ERROR_STREAM("[control] could not add gimbal" << msg->index[0] + 1 << " for planning");
          return;
        }
    }

  rolling_robot_model_->setGimbalPlanningFlag(msg->index[0], true);

  rolling_robot_model_->setCurrentGimbalPlanningAngle(msg->index[0], current_gimbal_angles.at(msg->index[0]));
  rolling_robot_model_->setFinalGimbalPlanningAngle(msg->index[0], msg->angle[0]);
  ROS_INFO_STREAM("[control] start planning for gimbal" << msg->index[0] + 1 << " current: " << current_gimbal_angles.at(msg->index[0]) << " target: " << msg->angle[0]);

  double gimbal_direction = angles::normalize_angle(msg->angle[0]) > 0 ? 1 : -1;
  if(gimbal_direction != gimbal_directions_.at(msg->index[0]))
    {
      ROS_WARN_STREAM("[control] gimbal" << msg->index[0] + 1 << " direction is changed from " << gimbal_directions_.at(msg->index[0]) << " to " << gimbal_direction);
      gimbal_directions_.at(msg->index[0]) = gimbal_direction;
    }
}
