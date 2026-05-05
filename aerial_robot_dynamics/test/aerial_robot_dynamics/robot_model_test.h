#pragma once

#include <aerial_robot_dynamics/robot_model.h>

namespace aerial_robot_dynamics
{
bool forwardDynamicsTest(PinocchioRobotModel& robot_model, bool verbose = false);
bool forwardDynamicsDerivativesTest(PinocchioRobotModel& robot_model, bool verbose = false);
bool inverseDynamicsTest(PinocchioRobotModel& robot_model, bool verbose = false);
bool computeTauExtByThrustDerivativeQDerivativesTest(PinocchioRobotModel& robot_model, bool verbose = false);
bool computeTauExtByThrustQDerivativeTest(PinocchioRobotModel& robot_model, bool verbose = false);
bool computeTauExtByThrustQDerivativeComparisonTest(PinocchioRobotModel& robot_model, bool verbose = false);
}  // namespace aerial_robot_dynamics
