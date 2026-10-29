#include <flying_humanoid/robot_model.h>

#include <pinocchio/parsers/urdf.hpp>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <regex>
#include <stdexcept>

namespace flying_humanoid
{
pinocchio::Model buildPinocchioModel(const std::string& robot_description, std::vector<RotorInfo>* rotors)
{
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(robot_description);
  if (!urdf_model)
    throw std::runtime_error("failed to parse robot_description");

  // only "rotor<number>" is a rotor; e.g. "rotor_arm1" is the gimbal frame used by the flight controller
  const std::regex rotor_name("rotor([0-9]+)");
  std::vector<std::pair<int, RotorInfo>> numbered_rotors;
  for (auto& joint : urdf_model->joints_)
  {
    std::smatch match;
    if (!std::regex_match(joint.first, match, rotor_name))
      continue;

    RotorInfo rotor;
    rotor.name = joint.first;
    rotor.min_thrust = joint.second->limits ? joint.second->limits->lower : 0.0;
    rotor.max_thrust = joint.second->limits ? joint.second->limits->upper : 0.0;
    rotor.direction = joint.second->axis.z > 0 ? 1 : -1;
    numbered_rotors.emplace_back(std::stoi(match[1]), rotor);

    // the rotor spins, but its angle does not matter for the dynamics
    joint.second->type = urdf::Joint::FIXED;
  }
  std::sort(numbered_rotors.begin(), numbered_rotors.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_model, pinocchio::JointModelFreeFlyer(), "root_joint", model);

  rotors->clear();
  for (auto& numbered_rotor : numbered_rotors)
  {
    RotorInfo& rotor = numbered_rotor.second;
    rotor.frame_id = model.getFrameId(rotor.name, pinocchio::FIXED_JOINT);
    rotors->push_back(rotor);
  }

  return model;
}
}  // namespace flying_humanoid
