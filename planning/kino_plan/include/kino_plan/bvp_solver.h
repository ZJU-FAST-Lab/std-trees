/*  
BSD 3-Clause License

Copyright (c) 2022, Hongkai Ye (kyle_yeh@163.com)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#ifndef _BVP_SOLVER_H_
#define _BVP_SOLVER_H_

#include "poly_traj_utils/traj_utils.hpp"
#include <Eigen/Eigen>

#define DOUBLE_INTEGRATOR 2
#define TRIPLE_INTEGRATOR 3
#define ACC_KNOWN 0
#define ACC_UNKNOWN 1
#define INITIAL_ACC_UNKNOWN 2
// #define HEURISTIC_TIME

namespace BVPSolver
{

  using Eigen::Vector2d;
  using Eigen::Vector3d;
  using Eigen::VectorXd;

  class IntegratorBVP
  {
  public:
    void init(int model)
    {
      model_ = model;
      if (model_ == DOUBLE_INTEGRATOR)
      {
        x0_ = Eigen::Matrix<double, 6, 1>::Zero();
        x1_ = Eigen::Matrix<double, 6, 1>::Zero();
      }
      else if (model_ == TRIPLE_INTEGRATOR)
      {
        x0_ = Eigen::Matrix<double, 9, 1>::Zero();
        x1_ = Eigen::Matrix<double, 9, 1>::Zero();
      }
    };

    void setRho(double rho)
    {
      rho_ = rho;
    };

    bool solve(const VectorXd &start, const VectorXd &goal, int type = ACC_UNKNOWN)
    {
      setBoundaries(start, goal);
      if (model_ == DOUBLE_INTEGRATOR)
      {
        return solveDouble();
      }
      else if (model_ == TRIPLE_INTEGRATOR)
      {
        if (type == ACC_UNKNOWN)
          // final state acc free
          return solveTripleAccUnknown();
        else if (type == INITIAL_ACC_UNKNOWN)
          return solveTripleInitialAccUnknown();
        else
          // final state acc fixed
          return solveTriple();
      }
      else
      {
        printf("Input model is neither double integrator nor triple.");
        return false;
      }
    };

    double estimateHeuristic(const VectorXd &start, const VectorXd &goal)
    {
      setBoundaries(start, goal);
      calTauStarDouble();
      return cost_star_;
    };

    double getTauStar()
    {
      return tau_star_;
    };

    double getCostStar()
    {
      return cost_star_;
    };

    void getCoeff(CoefficientMat &coeff)
    {
      coeff = coeff_;
    };

    void calCoeffFromTau(double tau, CoefficientMat &coeff);
    double calCostAccKnown(const VectorXd &x0, const VectorXd &x1, double T);
    double heuristicTimeAvgVel()
    {
      // double t(0.1);
      // Vector3d v0(x0_[3], x0_[4], x0_[5]), v1(x1_[3], x1_[4], x1_[5]);
      // double v_mag = (v0 + v1).norm() / 2.0;
      double dis = std::abs(x0_[0] - x1_[0]) + std::abs(x0_[1] - x1_[1]) + std::abs(x0_[2] - x1_[2]);
      // return std::max(t, dis/v_mag);
      return dis;
    }

  private:
    VectorXd x0_, x1_;
    double rho_;
    int model_;
    CoefficientMat coeff_;
    double tau_star_, cost_star_;

    bool calTauStarDouble();
    bool solveDouble();
    bool calTauStarTriple();
    bool solveTriple();

    bool calTauStarTripleAccUnknown();
    bool solveTripleAccUnknown();

    bool calTauStarTripleInitialAccUnknown();
    bool solveTripleInitialAccUnknown();

    bool calTauStarTripleVelAccUnknown();
    bool solveTripleVelAccUnknown();

    void setBoundaries(const VectorXd &start, const VectorXd &goal)
    {
      x0_ = start;
      x1_ = goal;
    };
  };

}
#endif