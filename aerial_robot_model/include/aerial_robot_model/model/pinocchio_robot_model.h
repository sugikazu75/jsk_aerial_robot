#pragma once

#include "pinocchio/fwd.hpp"

#include "pinocchio/parsers/urdf.hpp"

#include "pinocchio/algorithm/center-of-mass.hxx"
#include "pinocchio/algorithm/centroidal.hxx"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hxx"
#include "pinocchio/autodiff/casadi.hpp"
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3-tpl.hpp>
#include <pinocchio/spatial/inertia.hpp>


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// #include <casadi/casadi.hpp>
#include <CasadiEigen/CasadiEigen.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/LU>

#include <iostream>
#include <tinyxml.h>


namespace aerial_robot_model {
  class PinocchioRobotModel
  {
  public:
    PinocchioRobotModel();
    ~PinocchioRobotModel() = default;

    std::vector<casadi::SX> getRotorsOriginFromCog() {return rotors_origin_cog_;}
    std::vector<casadi::SX> getRotorsNormalFromCog() {return rotors_normal_cog_;}
    std::map<std::string, int> getJointIndexMap() {return joint_index_map_;}
    void updateRobotModel(const sensor_msgs::JointState& state);
    void updateRobotModelImpl(const sensor_msgs::JointState& state);

  private:
    std::string getRobotModelXml(const std::string param, ros::NodeHandle nh = ros::NodeHandle());

    void modelInit();
    void kinematicsInit();
    void inertialInit();
    void rotorInit();

    pinocchio::Model model_dbl_;
    pinocchio::Data data_dbl_;
    pinocchio::ModelTpl<casadi::SX> model_;
    pinocchio::DataTpl<casadi::SX> data_;

    std::vector<casadi::SX> rotors_origin_root_;
    std::vector<casadi::SX> rotors_origin_cog_;
    std::vector<casadi::SX> rotors_normal_root_;
    std::vector<casadi::SX> rotors_normal_cog_;

    casadi::SX q_cs_;
    Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> q_;
    casadi::DM q_dbl_;

    casadi::SX mass_;
    casadi::SX inertia_;
    pinocchio::SE3Tpl<casadi::SX> oMcog_;
    casadi::SX opcog_;

    std::map<std::string, int> joint_index_map_;

    int rotor_num_;
    std::string baselink_;
  };
}


Eigen::MatrixXd computeRealValue(casadi::SX y, casadi::SX x, Eigen::VectorXd x_dbl)
{
  casadi::DM ret = casadi::DM(y.size1(), y.size2());
  for(int i =  0; i < y.size1(); i++)
    {
      for(int j = 0; j < y.size2(); j++)
        {
          casadi::Function f = casadi::Function("f", {x}, {y(i, j)});
          casadi::DM y_dbl = f(eigenVectorToCasadiDm(x_dbl));
          ret(i, j) = y_dbl;
        }
    }
  return casadiDmToEigenMatrix(ret);
}

Eigen::MatrixXd computeRealValue(casadi::SX y, casadi::SX x, casadi::DM x_dbl)
{
  casadi::DM ret = casadi::DM(y.size1(), y.size2());
  for(int i =  0; i < y.size1(); i++)
    {
      for(int j = 0; j < y.size2(); j++)
        {
          casadi::Function f = casadi::Function("f", {x}, {y(i, j)});
          casadi::DM y_dbl = f(x_dbl);
          ret(i, j) = y_dbl;
        }
    }
  return casadiDmToEigenMatrix(ret);
}