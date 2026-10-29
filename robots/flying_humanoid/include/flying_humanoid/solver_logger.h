// -*- mode: c++ -*-
#pragma once

#include <crocoddyl/core/solver-base.hpp>

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace flying_humanoid
{
// Records the solver quantities of every iteration, to see why a solve does not converge.
class SolverLogger : public crocoddyl::CallbackAbstractTpl<double>
{
public:
  struct Iteration
  {
    int iter;
    double cost, merit, stop, grad, preg, dreg, step, ffeas, gfeas, hfeas, dV_exp, dV;
    int max_defect_node;  // node with the largest dynamics defect (gap)
    double max_defect;
  };

  // required by the scalar casts of crocoddyl; only the double version is used here
  std::shared_ptr<crocoddyl::CallbackBase> cloneAsDouble() const override
  {
    return std::make_shared<SolverLogger>(*this);
  }
  std::shared_ptr<crocoddyl::CallbackBase> cloneAsFloat() const override
  {
    return nullptr;
  }

  void operator()(crocoddyl::SolverAbstractTpl<double>& solver) override
  {
    Iteration it;
    it.iter = solver.get_iter();
    it.cost = solver.get_cost();
    it.merit = solver.get_merit();
    it.stop = solver.get_stop();
    it.grad = std::abs(solver.get_DV()[1]);
    it.preg = solver.get_preg();
    it.dreg = solver.get_dreg();
    it.step = solver.get_steplength();
    it.ffeas = solver.get_ffeas();
    it.gfeas = solver.get_gfeas();
    it.hfeas = solver.get_hfeas();
    it.dV_exp = solver.get_dVexp();
    it.dV = solver.get_dV();
    it.max_defect_node = -1;
    it.max_defect = 0.0;
    const auto& fs = solver.get_fs();
    for (std::size_t t = 0; t < fs.size(); ++t)
    {
      const double defect = fs[t].lpNorm<Eigen::Infinity>();
      if (defect > it.max_defect)
      {
        it.max_defect = defect;
        it.max_defect_node = t;
      }
    }
    iterations_.push_back(it);
  }

  void clear()
  {
    iterations_.clear();
  }
  const std::vector<Iteration>& getIterations() const
  {
    return iterations_;
  }

  bool save(const std::string& filename) const
  {
    std::ofstream ofs(filename);
    if (!ofs)
      return false;
    ofs << "iter,cost,merit,stop,grad,preg,dreg,step,ffeas,gfeas,hfeas,dV_exp,dV,max_defect_node,max_defect\n";
    for (const auto& it : iterations_)
      ofs << it.iter << "," << it.cost << "," << it.merit << "," << it.stop << "," << it.grad << "," << it.preg << ","
          << it.dreg << "," << it.step << "," << it.ffeas << "," << it.gfeas << "," << it.hfeas << "," << it.dV_exp
          << "," << it.dV << "," << it.max_defect_node << "," << it.max_defect << "\n";
    return true;
  }

private:
  std::vector<Iteration> iterations_;
};
}  // namespace flying_humanoid
