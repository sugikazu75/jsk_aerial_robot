// -*- mode: c++ -*-
#pragma once

#include <pinocchio/fwd.hpp>  // should be included before any other pinocchio headers
#include <pinocchio/multibody/model.hpp>

#include <string>
#include <vector>

namespace flying_humanoid
{
struct RotorInfo
{
  std::string name;                // "rotor1", "rotor2", ...
  pinocchio::FrameIndex frame_id;  // fixed frame placed at the rotor
  double min_thrust;               // [N], lower limit of the rotor joint
  double max_thrust;               // [N], upper limit of the rotor joint
  int direction;                   // z component of the rotor joint axis (+1 or -1)
};

// Build the floating-base pinocchio model from robot_description.
// The rotor joints ("rotor1", "rotor2", ...) are turned into fixed frames, and their limit and axis are returned
// in rotors, ordered by the rotor number. The floating base joint is named "root_joint".
pinocchio::Model buildPinocchioModel(const std::string& robot_description, std::vector<RotorInfo>* rotors);
}  // namespace flying_humanoid
