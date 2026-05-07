#include <aerial_robot_dynamics/robot_model.h>
#include <aerial_robot_dynamics/robot_model_ros.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sstream>
#include <string>
#include <vector>

namespace py = pybind11;

namespace
{
using aerial_robot_dynamics::PinocchioRobotModel;
using aerial_robot_dynamics::PinocchioRobotModelRos;

Eigen::VectorXd copyVector(const Eigen::Ref<const Eigen::VectorXd>& value)
{
  return Eigen::VectorXd(value);
}

void checkVectorSize(const Eigen::VectorXd& value, Eigen::Index expected, const std::string& name)
{
  if (value.size() == expected)
    return;

  std::ostringstream message;
  message << name << " must have size " << expected << ", got " << value.size();
  throw py::value_error(message.str());
}

void checkStateSizes(const PinocchioRobotModel& robot_model, const Eigen::VectorXd& q, const Eigen::VectorXd& v)
{
  checkVectorSize(q, robot_model.getModel()->nq, "q");
  checkVectorSize(v, robot_model.getModel()->nv, "v");
}

std::vector<Eigen::VectorXd> forceVectorToEigen(const std::vector<pinocchio::Force>& forces)
{
  std::vector<Eigen::VectorXd> result;
  result.reserve(forces.size());
  for (const auto& force : forces)
    result.push_back(force.toVector());
  return result;
}

std::vector<Eigen::Matrix4d> se3VectorToHomogeneous(const std::vector<pinocchio::SE3>& poses)
{
  std::vector<Eigen::Matrix4d> result;
  result.reserve(poses.size());
  for (const auto& pose : poses)
    result.push_back(pose.toHomogeneousMatrix());
  return result;
}

void ensureRosInitialized(const std::string& node_name)
{
  if (ros::isInitialized())
    return;

  ros::M_string remappings;
  ros::init(remappings, node_name, ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
}

std::shared_ptr<PinocchioRobotModelRos> createPinocchioRobotModelRos(const std::string& ros_namespace,
                                                                     const std::string& node_name)
{
  ensureRosInitialized(node_name);
  ros::NodeHandle nh(ros_namespace);
  return std::make_shared<PinocchioRobotModelRos>(nh);
}
}  // namespace

PYBIND11_MODULE(_robot_model, m)
{
  m.doc() = "Python bindings for aerial_robot_dynamics robot models";

  auto forward_dynamics = [](PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
                             const Eigen::Ref<const Eigen::VectorXd>& v, const Eigen::Ref<const Eigen::VectorXd>& tau,
                             const Eigen::Ref<const Eigen::VectorXd>& thrust) {
    Eigen::VectorXd q_copy = copyVector(q);
    Eigen::VectorXd v_copy = copyVector(v);
    Eigen::VectorXd tau_copy = copyVector(tau);
    Eigen::VectorXd thrust_copy = copyVector(thrust);
    checkStateSizes(self, q_copy, v_copy);
    checkVectorSize(tau_copy, self.getModel()->nv, "tau");
    checkVectorSize(thrust_copy, self.getRotorNum(), "thrust");
    return self.forwardDynamics(q_copy, v_copy, tau_copy, thrust_copy);
  };

  auto forward_dynamics_derivatives = [](PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
                                         const Eigen::Ref<const Eigen::VectorXd>& v,
                                         const Eigen::Ref<const Eigen::VectorXd>& tau,
                                         const Eigen::Ref<const Eigen::VectorXd>& thrust) {
    Eigen::VectorXd q_copy = copyVector(q);
    Eigen::VectorXd v_copy = copyVector(v);
    Eigen::VectorXd tau_copy = copyVector(tau);
    Eigen::VectorXd thrust_copy = copyVector(thrust);
    checkStateSizes(self, q_copy, v_copy);
    checkVectorSize(tau_copy, self.getModel()->nv, "tau");
    checkVectorSize(thrust_copy, self.getRotorNum(), "thrust");
    return self.forwardDynamicsDerivatives(q_copy, v_copy, tau_copy, thrust_copy);
  };

  auto inverse_dynamics = [](PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
                             const Eigen::Ref<const Eigen::VectorXd>& v, const Eigen::Ref<const Eigen::VectorXd>& a) {
    Eigen::VectorXd q_copy = copyVector(q);
    Eigen::VectorXd v_copy = copyVector(v);
    Eigen::VectorXd a_copy = copyVector(a);
    checkStateSizes(self, q_copy, v_copy);
    checkVectorSize(a_copy, self.getModel()->nv, "a");
    Eigen::VectorXd solution;
    const bool ok = self.inverseDynamics(q_copy, v_copy, a_copy, solution);
    return py::make_tuple(ok, solution);
  };

  py::class_<PinocchioRobotModel, std::shared_ptr<PinocchioRobotModel>>(m, "PinocchioRobotModel")
      .def(py::init<std::string, std::string, bool, double>(), py::arg("robot_description"),
           py::arg("pinocchio_robot_description"), py::arg("is_floating_base") = true,
           py::arg("thrust_hessian_weight") = 1.0)
      .def_property_readonly("robot_description", &PinocchioRobotModel::getRobotDescription)
      .def_property_readonly("pinocchio_robot_description", &PinocchioRobotModel::getPinocchioRobotDescription)
      .def_property_readonly("is_floating_base", &PinocchioRobotModel::getIsFloatingBase)
      .def_property_readonly("rotor_num", &PinocchioRobotModel::getRotorNum)
      .def_property_readonly("m_f_rate", &PinocchioRobotModel::getMFRate)
      .def_property_readonly("latest_id_solve_time", &PinocchioRobotModel::getLatestIdSolveTime)
      .def_property_readonly("joint_torque_limits", &PinocchioRobotModel::getJointTorqueLimits)
      .def_property_readonly("thrust_upper_limits", &PinocchioRobotModel::getThrustUpperLimits)
      .def_property_readonly("thrust_lower_limits", &PinocchioRobotModel::getThrustLowerLimits)
      .def_property_readonly("thrust_hessian_weight", &PinocchioRobotModel::getThrustHessianWeight)
      .def_property_readonly("rotor_frame_indices", &PinocchioRobotModel::getRotorFrameIndices)
      .def_property_readonly("rotor_directions",
                             [](const PinocchioRobotModel& self) {
                               std::vector<int> directions;
                               directions.reserve(self.getRotorNum());
                               for (int i = 0; i < self.getRotorNum(); ++i)
                                 directions.push_back(self.getRotorDirection(i));
                               return directions;
                             })
      .def_property_readonly(
          "joint_M_rotors",
          [](const PinocchioRobotModel& self) { return se3VectorToHomogeneous(self.getJointMRotors()); })
      .def_property_readonly("nq", [](const PinocchioRobotModel& self) { return self.getModel()->nq; })
      .def_property_readonly("nv", [](const PinocchioRobotModel& self) { return self.getModel()->nv; })
      .def_property_readonly("njoints", [](const PinocchioRobotModel& self) { return self.getModel()->njoints; })
      .def_property_readonly("nframes", [](const PinocchioRobotModel& self) { return self.getModel()->nframes; })
      .def("get_rotor_direction", &PinocchioRobotModel::getRotorDirection, py::arg("index"))
      .def("get_reset_configuration", &PinocchioRobotModel::getResetConfiguration)
      .def("forward_dynamics", forward_dynamics, py::arg("q"), py::arg("v"), py::arg("tau"), py::arg("thrust"))
      .def("forward_dynamics_derivatives", forward_dynamics_derivatives, py::arg("q"), py::arg("v"), py::arg("tau"),
           py::arg("thrust"))
      .def("inverse_dynamics", inverse_dynamics, py::arg("q"), py::arg("v"), py::arg("a"))
      .def(
          "compute_f_ext_by_thrust",
          [](PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& thrust) {
            Eigen::VectorXd thrust_copy = copyVector(thrust);
            checkVectorSize(thrust_copy, self.getRotorNum(), "thrust");
            return forceVectorToEigen(self.computeFExtByThrust(thrust_copy));
          },
          py::arg("thrust"))
      .def(
          "compute_tau_ext_by_thrust",
          [](PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
             const Eigen::Ref<const Eigen::VectorXd>& thrust) {
            Eigen::VectorXd q_copy = copyVector(q);
            Eigen::VectorXd thrust_copy = copyVector(thrust);
            checkVectorSize(q_copy, self.getModel()->nq, "q");
            checkVectorSize(thrust_copy, self.getRotorNum(), "thrust");
            return self.computeTauExtByThrust(q_copy, thrust_copy);
          },
          py::arg("q"), py::arg("thrust"))
      .def(
          "compute_tau_ext_by_thrust_derivative",
          [](PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q) {
            Eigen::VectorXd q_copy = copyVector(q);
            checkVectorSize(q_copy, self.getModel()->nq, "q");
            return self.computeTauExtByThrustDerivative(q_copy);
          },
          py::arg("q"))
      .def(
          "compute_tau_ext_by_thrust_q_derivative",
          [](PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q,
             const Eigen::Ref<const Eigen::VectorXd>& thrust) {
            Eigen::VectorXd q_copy = copyVector(q);
            Eigen::VectorXd thrust_copy = copyVector(thrust);
            checkVectorSize(q_copy, self.getModel()->nq, "q");
            checkVectorSize(thrust_copy, self.getRotorNum(), "thrust");
            return self.computeTauExtByThrustQDerivative(q_copy, thrust_copy);
          },
          py::arg("q"), py::arg("thrust"))
      .def(
          "compute_tau_ext_by_thrust_derivative_q_derivatives",
          [](PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q) {
            Eigen::VectorXd q_copy = copyVector(q);
            checkVectorSize(q_copy, self.getModel()->nq, "q");
            return self.computeTauExtByThrustDerivativeQDerivatives(q_copy);
          },
          py::arg("q"))
      .def(
          "compute_tau_ext_by_thrust_derivative_q_derivatives_num",
          [](PinocchioRobotModel& self, const Eigen::Ref<const Eigen::VectorXd>& q) {
            Eigen::VectorXd q_copy = copyVector(q);
            checkVectorSize(q_copy, self.getModel()->nq, "q");
            return self.computeTauExtByThrustDerivativeQDerivativesNum(q_copy);
          },
          py::arg("q"))
      .def("forwardDynamics", forward_dynamics, py::arg("q"), py::arg("v"), py::arg("tau"), py::arg("thrust"))
      .def("forwardDynamicsDerivatives", forward_dynamics_derivatives, py::arg("q"), py::arg("v"), py::arg("tau"),
           py::arg("thrust"))
      .def("inverseDynamics", inverse_dynamics, py::arg("q"), py::arg("v"), py::arg("a"));

  py::class_<PinocchioRobotModelRos, std::shared_ptr<PinocchioRobotModelRos>>(m, "PinocchioRobotModelRos")
      .def(py::init(&createPinocchioRobotModelRos), py::arg("ros_namespace") = "",
           py::arg("node_name") = "aerial_robot_dynamics_py")
      .def_property_readonly("pinocchio_robot_model", &PinocchioRobotModelRos::getPinocchioRobotModel)
      .def("get_pinocchio_robot_model", &PinocchioRobotModelRos::getPinocchioRobotModel)
      .def("getPinocchioRobotModel", &PinocchioRobotModelRos::getPinocchioRobotModel);
}
