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
#pragma once

#include "occ_grid/pos_checker.h"
#include "visualization_utils/visualization_utils.h"
#include "poly_opt/lbfgs.hpp"
#include "poly_traj_utils/traj_utils.hpp"
#include <Eigen/Eigen>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <ros/ros.h>

using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::VectorXd;
using Eigen::Vector3d;
using std::vector; 
#define AS_PARENT 0
#define AS_CHILD 1

#define NO_DEFORM 0
#define ONE_NODE 1
#define TRUNK_NODES 2
#define BRANCH_NODES 3
#define TREE_NODES 4
#define SMALL_BRANCH_NODES 5

class BranchOpt
{
private:
  // decision variables, px, py, pz, vx, vy, vz, ax, ay, az, Tc1, Tc2, ..., Tcm, Tp
  double *x_;
  // parent state
  VectorXd state_p_;
  // children states
  vector<VectorXd> state_c_;

  vector<int> children_descendant_num_plus_one_;
  int state_size_, children_size_;
  int self_descendant_num_plus_one_;

  double rho_;
  double vel_limit_, acc_limit_, jerk_limit_;
  double squared_vel_limit_, squared_acc_limit_, squared_jerk_limit_;
  int K_;
  vector<int> kpos_;
  double rho_pos_, rho_vel_, rho_acc_;
  double dist_tolerance_;

  tgk_planner::PosChecker::Ptr pos_checker_ptr_;
  VisualRviz::Ptr vis_ptr_;

public:
  void setPosChecker(const tgk_planner::PosChecker::Ptr& checker)
  {
    pos_checker_ptr_ = checker;
  };
  void setVisualizer(const VisualRviz::Ptr &vis)
  {
    vis_ptr_ = vis;
  }

  BranchOpt(const ros::NodeHandle &node) : x_(NULL)
  {
    node.param("trunk_opt/rho", rho_, 0.0);
    node.param("trunk_opt/vel_limit", vel_limit_, 0.0);
    node.param("trunk_opt/acc_limit", acc_limit_, 0.0);
    node.param("trunk_opt/jerk_limit", jerk_limit_, 0.0);
    node.param("trunk_opt/discretization", K_, 0);
    node.param("trunk_opt/rho_pos", rho_pos_, 0.0);
    node.param("trunk_opt/rho_vel", rho_vel_, 0.0);
    node.param("trunk_opt/rho_acc", rho_acc_, 0.0);
    node.param("trunk_opt/dist_tolerance", dist_tolerance_, 0.0);
    squared_vel_limit_ = vel_limit_ * vel_limit_;
    squared_acc_limit_ = acc_limit_ * acc_limit_;
    squared_jerk_limit_ = jerk_limit_ * jerk_limit_;
    ROS_WARN_STREAM("[trunk_opt] param: vel_limit: " << vel_limit_);
    ROS_WARN_STREAM("[trunk_opt] param: acc_limit: " << acc_limit_);
    ROS_WARN_STREAM("[trunk_opt] param: jerk_limit: " << jerk_limit_);
    ROS_WARN_STREAM("[trunk_opt] param: discretization: " << K_);
    ROS_WARN_STREAM("[trunk_opt] param: rho_pos: " << rho_pos_);
    ROS_WARN_STREAM("[trunk_opt] param: rho_vel: " << rho_vel_);
    ROS_WARN_STREAM("[trunk_opt] param: rho_acc: " << rho_acc_);
  }

  ~BranchOpt()
  {
  }

  bool optimizeBranch(VectorXd &node_state, 
                      const VectorXd &parent_state, 
                      double &parent_T, 
                      int parent_descendant_num_plus_one, 
                      const vector<VectorXd> &children_states, 
                      vector<double> &children_Ts, 
                      const vector<int> &children_descendant_num_plus_one) 
  {
    bool res(false);
    state_size_ = node_state.size();
    children_size_ = children_Ts.size();
    int N = state_size_ + children_size_ + 1;

    state_p_ = parent_state;
    state_c_ = children_states;
    self_descendant_num_plus_one_ = parent_descendant_num_plus_one;
    children_descendant_num_plus_one_ = children_descendant_num_plus_one;
    
    kpos_.resize(children_size_ + 1);
    for (int i=0; i<children_size_; ++i)
    {
      kpos_[i] = std::max(2.0, std::min(ceil(children_Ts[i] / 0.03), 200.0));
    }
    kpos_[children_size_] = std::max(2.0, std::min(ceil(parent_T / 0.03), 200.0));

    // ROS_WARN_STREAM("node_state: " << node_state.transpose());
    // ROS_WARN_STREAM("children_size_: " << children_size_);
    // ROS_WARN_STREAM("state_p_: " << state_p_.transpose());
    // ROS_WARN_STREAM("parent_T: " << parent_T);

    /* Initialize the variables. */
    x_ = new double[N];
    for (int i=0; i<state_size_; i++)
    {
      x_[i] = node_state[i];
    }
    for (int i=0; i<children_size_; i++)
    {
      // ROS_WARN_STREAM("children_states: " << children_states[i].transpose());
      // ROS_WARN_STREAM("children_Ts: " << children_Ts[i]);
      x_[i + state_size_] = log(children_Ts[i]);
      // ROS_WARN_STREAM("initial T children: " << children_Ts[i]);
    }
    // ROS_WARN_STREAM("initial T parent: " << parent_T);
    x_[N - 1] = log(parent_T);

    int opt_ret = run(N);
    if (opt_ret < 0) 
    {
      res = false;
    }
    else
    {
      for (int i=0; i<state_size_; i++)
      {
        node_state[i] = x_[i];
      }
      for (int i=0; i<children_size_; i++)
      {
        children_Ts[i] = exp(x_[i + state_size_]);
      }
      parent_T = exp(x_[N - 1]);
      res = true;
    }

    delete[] x_;
    return res;
  }

private:
  int run(int N)
  {
    double fx;
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.past = 3;
    lbfgs_params.g_epsilon = 0.01;
    // lbfgs_params.min_step = 1e-32;
    // int ret = lbfgs::lbfgs_optimize(N, x_, &fx, _evaluate, NULL, _progress, this, &lbfgs_params);
    int ret = lbfgs::lbfgs_optimize(N, x_, &fx, _evaluate, NULL, NULL, this, &lbfgs_params);

    /* Report the result. */
    printf("L-BFGS optimization terminated with status code = %d\n", ret);
    // printf("  fx = %f\n", fx);

    return ret;
  } 

  static double _evaluate(void *instance,
                          const double *x,
                          double *g,
                          const int n)
  {
    return reinterpret_cast<BranchOpt *>(instance)->evaluate(x, g, n);
  }

  double evaluate(const double *x,
                  double *g,
                  const int n)
  {
    /* for current iteration */
    double obj = 0.0;
    // state: [px py pz vx vy vz ax ay az]'
    Eigen::Map<const VectorXd> D(x, state_size_); 
    Eigen::Map<VectorXd> g_D(g, state_size_);
    g_D.setZero();
    // time durations: [Tc1 Tc2 ... Tcm Tp]', children's and the last the parent's
    Eigen::Map<const VectorXd> Tau(x + state_size_, children_size_ + 1); 
    Eigen::Map<VectorXd> g_Tau(g + state_size_, children_size_ + 1);
    g_Tau.setZero();
    /* segment: curr_node to its parent */
    obj += calSmoothCostAddGrad(state_p_, D, Tau[children_size_], g_D, g_Tau[children_size_], AS_CHILD, self_descendant_num_plus_one_);
    obj += calTimeItgPntAddGrad(state_p_, D, Tau[children_size_], g_D, g_Tau[children_size_], AS_CHILD, kpos_[children_size_], self_descendant_num_plus_one_);

    /* segments: curr_node to its children */
    for (int i=0; i<children_size_; ++i)
    {
      obj += calSmoothCostAddGrad(D, state_c_[i], Tau[i], g_D, g_Tau[i], AS_PARENT, children_descendant_num_plus_one_[i]);
      obj += calTimeItgPntAddGrad(D, state_c_[i], Tau[i], g_D, g_Tau[i], AS_PARENT, kpos_[i], children_descendant_num_plus_one_[i]);
    }

    return obj;
  }

  double calSmoothCostAddGrad(const VectorXd &x0, const VectorXd &x1, double Tau, Eigen::Map<VectorXd> grad_d, double &grad_Tau, bool type, double weight = 1)
  {
    double t1 = 3*(x0[6]*x0[6] + x0[7]*x0[7] + x0[8]*x0[8] + x1[6]*x1[6] + x1[7]*x1[7] + x1[8]*x1[8]);
    double t2 = t1 - 2*(x0[6]*x1[6] + x0[7]*x1[7] + x0[8]*x1[8]);
    double t3 = 3*(x0[3]*x0[6] + x0[4]*x0[7] + x0[5]*x0[8] - x1[3]*x1[6] - x1[4]*x1[7] - x1[5]*x1[8]);
    double t4 = t3 + 2*(x0[6]*x1[3] + x0[7]*x1[4] + x0[8]*x1[5] - x0[3]*x1[6] - x0[4]*x1[7] - x0[5]*x1[8]);
    double t5 = 8*(x0[3]*x0[3] + x0[4]*x0[4] + x0[5]*x0[5] + x1[3]*x1[3] + x1[4]*x1[4] + x1[5]*x1[5]);
    double t6 = t5 + 5*((x0[0]-x1[0])*(x0[6]-x1[6]) + (x0[1]-x1[1])*(x0[7]-x1[7]) + (x0[2]-x1[2])*(x0[8]-x1[8]));
    double t7 = t6 + 14*(x0[3]*x1[3] + x0[4]*x1[4] + x0[5]*x1[5]);
    double t8 = (x0[0]-x1[0])*(x0[3]+x1[3]) + (x0[1]-x1[1])*(x0[4]+x1[4]) + (x0[2]-x1[2])*(x0[5]+x1[5]);
    double t9 = (x0[0]-x1[0])*(x0[0]-x1[0]) + (x0[1]-x1[1])*(x0[1]-x1[1]) + (x0[2]-x1[2])*(x0[2]-x1[2]);

    double T(exp(Tau));
    double T2 = T*T;
    double T3 = T2*T;
    double T4 = T3*T;
    double T5 = T4*T;
    double T6 = T5*T;

    double cost = (T6 + rho_*(720*t9 + 720*T*t8 + 24*T2*t7 + 24*T3*t4 + 3*T4*t2)) / T5;
    grad_Tau += weight * (T6 - rho_*(3600*t9 + 2880*T*t8 + 72*T2*t7 + 48*T3*t4 + 3*T4*t2)) / T6 * exp(Tau);
    // ROS_ERROR_STREAM("smt T grad: " << grad_T);

    if (type == AS_PARENT)
    {
      grad_d[0] += 120*rho_*(12*(x0[0]-x1[0]) + 6*T*(x0[3]+x1[3]) + T2*(x0[6]-x1[6])) / T5;
      grad_d[1] += 120*rho_*(12*(x0[1]-x1[1]) + 6*T*(x0[4]+x1[4]) + T2*(x0[7]-x1[7])) / T5;
      grad_d[2] += 120*rho_*(12*(x0[2]-x1[2]) + 6*T*(x0[5]+x1[5]) + T2*(x0[8]-x1[8])) / T5;
      grad_d[3] += 24*rho_*(30*(x0[0]-x1[0]) + T*(16*x0[3]+14*x1[3]) + T2*(3*x0[6]-2*x1[6])) / T4;
      grad_d[4] += 24*rho_*(30*(x0[1]-x1[1]) + T*(16*x0[4]+14*x1[4]) + T2*(3*x0[7]-2*x1[7])) / T4;
      grad_d[5] += 24*rho_*(30*(x0[2]-x1[2]) + T*(16*x0[5]+14*x1[5]) + T2*(3*x0[8]-2*x1[8])) / T4;
      grad_d[6] += 6*rho_*(20*(x0[0]-x1[0]) + T*(12*x0[3]+8*x1[3]) + T2*(3*x0[6]-x1[6])) / T3;
      grad_d[7] += 6*rho_*(20*(x0[1]-x1[1]) + T*(12*x0[4]+8*x1[4]) + T2*(3*x0[7]-x1[7])) / T3;
      grad_d[8] += 6*rho_*(20*(x0[2]-x1[2]) + T*(12*x0[5]+8*x1[5]) + T2*(3*x0[8]-x1[8])) / T3;
    }
    else if (type == AS_CHILD)
    {
      grad_d[0] += -120*rho_*(12*(x0[0]-x1[0]) + 6*T*(x0[3]+x1[3]) + T2*(x0[6]-x1[6])) / T5;
      grad_d[1] += -120*rho_*(12*(x0[1]-x1[1]) + 6*T*(x0[4]+x1[4]) + T2*(x0[7]-x1[7])) / T5;
      grad_d[2] += -120*rho_*(12*(x0[2]-x1[2]) + 6*T*(x0[5]+x1[5]) + T2*(x0[8]-x1[8])) / T5;
      grad_d[3] += 24*rho_*(30*(x0[0]-x1[0]) + T*(14*x0[3]+16*x1[3]) + T2*(2*x0[6]-3*x1[6])) / T4;
      grad_d[4] += 24*rho_*(30*(x0[1]-x1[1]) + T*(14*x0[4]+16*x1[4]) + T2*(2*x0[7]-3*x1[7])) / T4;
      grad_d[5] += 24*rho_*(30*(x0[2]-x1[2]) + T*(14*x0[5]+16*x1[5]) + T2*(2*x0[8]-3*x1[8])) / T4;
      grad_d[6] += -6*rho_*(20*(x0[0]-x1[0]) + T*(8*x0[3]+12*x1[3]) + T2*(x0[6]-3*x1[6])) / T3;
      grad_d[7] += -6*rho_*(20*(x0[1]-x1[1]) + T*(8*x0[4]+12*x1[4]) + T2*(x0[7]-3*x1[7])) / T3;
      grad_d[8] += -6*rho_*(20*(x0[2]-x1[2]) + T*(8*x0[5]+12*x1[5]) + T2*(x0[8]-3*x1[8])) / T3;
    }
    grad_d *= weight;

    return cost * weight;
  }

  double calTimeItgPntAddGrad(const VectorXd &x0, const VectorXd &x1, double Tau, Eigen::Map<VectorXd> grad_d, double &grad_Tau, bool type, int &kpos, double weight = 1) 
  {
    CoefficientMat coeff; /* [cx5,cx4,cx3,cx2,cx1,cx0; 
                              cy5,cy4,cy3,cy2,cy1,cy0; 
                              cz5,cz4,cz3,cz2,cz1,cz0] */
    double T(exp(Tau));
    double T2(T*T), T3(T2*T), T4(T3*T), T5(T4*T);
    coeff.col(0) = (0.5 * (x1.segment(6, 3) - x0.segment(6, 3)) * T2 -
                    3.0 * (x0.segment(3, 3) + x1.segment(3, 3)) * T +
                    6.0 * (x1.segment(0, 3) - x0.segment(0, 3))) / T5;
    coeff.col(1) = ((-x1.segment(6, 3) + 1.5 * x0.segment(6, 3)) * T2 +
                    (8.0 * x0.segment(3, 3) + 7.0 * x1.segment(3, 3)) * T +
                    15.0 * (-x1.segment(0, 3) + x0.segment(0, 3))) / T4;
    coeff.col(2) = ((0.5 * x1.segment(6, 3) - 1.5 * x0.segment(6, 3)) * T2 -
                    (6.0 * x0.segment(3, 3) + 4.0 * x1.segment(3, 3)) * T +
                    10.0 * (x1.segment(0, 3) - x0.segment(0, 3))) / T3;
    coeff.col(3) = 0.5 * x0.segment(6, 3);
    coeff.col(4) = x0.segment(3, 3);
    coeff.col(5) = x0.segment(0, 3);

    Matrix<double, 3, 3> grad_Jp_to_d = Matrix<double, 3, 3>::Zero(3, 3);
    Matrix<double, 3, 3> grad_Jv_to_d = Matrix<double, 3, 3>::Zero(3, 3);
    Matrix<double, 3, 3> grad_Ja_to_d = Matrix<double, 3, 3>::Zero(3, 3);
    Vector3d grad_Jp_to_p, grad_Jv_to_v, grad_Ja_to_a;
    Vector3d grad_p_to_t, grad_v_to_t, grad_a_to_t;
    Matrix<double, 3, 6> grad_Jp_to_c, grad_Jv_to_c, grad_Ja_to_c;
    Matrix<double, 6, 3> grad_c_to_d = Matrix<double, 6, 3>::Zero(6, 3);
    Matrix<double, 3, 6> grad_c_to_t = Matrix<double, 3, 6>::Zero(3, 6);

    double grad_Jp_to_t(0.0), grad_Jv_to_t(0.0), grad_Ja_to_t(0.0);
    if (type == AS_PARENT)
    {
      grad_c_to_d << -6./T5, -3./T4, -0.5/T3,
                      15./T4, 8./T3, 1.5/T2,
                      -10/T3, -6./T2, -1.5/T,
                      0, 0, 0.5, 
                      0, 1, 0, 
                      1, 0, 0;
    }
    else if (type == AS_CHILD)
    {
      grad_c_to_d << 6./T5, -3./T4, 0.5/T3, 
                     -15./T4, 7./T3, -1/T2, 
                     10./T3, -4./T2, 0.5/T, 
                     0, 0, 0, 
                     0, 0, 0, 
                     0, 0, 0;
    }
    for (int i=0; i<3; ++i)
    {
      double tmp = x0[i] - x1[i];
      grad_c_to_t(i, 0) = (30*tmp + T*(1.5*x0[6+i]*T - 1.5*x1[6+i]*T + 12*x0[3+i] + 12*x1[3+i])) / T5 / T;
      grad_c_to_t(i, 1) = (-60*tmp + T*(-3*x0[6+i]*T + 2*x1[6+i]*T - 24*x0[3+i] - 21*x1[3+i])) / T5;
      grad_c_to_t(i, 2) = (30*tmp + T*(1.5*x0[6+i]*T - 0.5*x1[6+i]*T + 12*x0[3+i] + 8*x1[3+i])) / T4;
    }
    double step_pos, omega;
    Vector3d pos, vel, acc, jerk;
    Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
    double t1(0.0), t2, t3, t4, t5;
    int K_pos = kpos;
    step_pos = T / K_pos;
    double cost_pos(0.0), cost_vel(0.0), cost_acc(0.0);
    double cost_tmp(0.0); 
    for (int j = 0; j <= K_pos; ++j, t1 += step_pos) 
    {
      t2 = t1 * t1;
      t3 = t2 * t1;
      t4 = t2 * t2;
      t5 = t4 * t1;
      beta0 << t5, t4, t3, t2, t1, 1.0;
      beta1 << 5.0 * t4, 4.0 * t3, 3.0 * t2, 2.0 * t1, 1.0, 0.0;
      
      pos = coeff * beta0;

      omega = (j == 0 || j == K_pos) ? 0.5 : 1.0;
      if (collisionCostGrad(pos, grad_Jp_to_p, cost_tmp)) 
      {
        // ROS_INFO_STREAM("AS_CHILD: " << type << ", j: " << j << ", T: " << T << ", pos: " << pos.transpose() << ", grad: " << grad_Jp_to_p.transpose());
        grad_Jp_to_c = grad_Jp_to_p * beta0.transpose();
        grad_Jp_to_d += omega * grad_Jp_to_c * grad_c_to_d;
        grad_p_to_t = grad_c_to_t * beta0 + coeff * beta1 * j / K_pos;
        grad_Jp_to_t += omega * (cost_tmp / K_pos + step_pos * grad_Jp_to_p.transpose() * grad_p_to_t);
        cost_pos += omega * cost_tmp;
      }
    }

    t1 = 0.0;
    double step_higher_deri = T / K_;
    for (int j = 0; j <= K_; ++j, t1 += step_higher_deri) 
    {
      t2 = t1 * t1;
      t3 = t2 * t1;
      t4 = t2 * t2;
      t5 = t4 * t1;
      beta1 << 5.0 * t4, 4.0 * t3, 3.0 * t2, 2.0 * t1, 1.0, 0.0;
      beta2 << 20.0 * t3, 12.0 * t2, 6.0 * t1, 2.0, 0.0, 0.0;
      beta3 << 60.0 * t2, 24.0 * t1, 6.0, 0.0, 0.0, 0.0;
      vel = coeff * beta1;
      acc = coeff * beta2;
      omega = (j == 0 || j == K_) ? 0.5 : 1.0;
      if (highDerivativeCostGrad(vel, squared_vel_limit_, grad_Jv_to_v, cost_tmp)) 
      {
        // ROS_INFO_STREAM("AS_CHILD: " << type << ", j: " << j << ", T: " << T << ", vel: " << vel.norm() << ", grad: " << grad_Jv_to_v.transpose());
        grad_Jv_to_c = grad_Jv_to_v * beta1.transpose();
        grad_Jv_to_d += omega * grad_Jv_to_c * grad_c_to_d;
        grad_v_to_t = grad_c_to_t * beta1 + coeff * beta2 * j / K_;
        grad_Jv_to_t += omega * (cost_tmp / K_ + step_higher_deri * grad_Jv_to_v.transpose() * grad_v_to_t);
        cost_vel += omega * cost_tmp;
      }
      if (highDerivativeCostGrad(acc, squared_acc_limit_, grad_Ja_to_a, cost_tmp)) 
      {
        // ROS_INFO_STREAM("AS_CHILD: " << type << ", j: " << j << ", T: " << T << ", acc: " << acc.norm() << ", grad: " << grad_Ja_to_a.transpose());
        grad_Ja_to_c = grad_Ja_to_a * beta2.transpose();
        grad_Ja_to_d += omega * grad_Ja_to_c * grad_c_to_d;
        grad_a_to_t = grad_c_to_t * beta2 + coeff * beta3 * j / K_;
        grad_Ja_to_t += omega * (cost_tmp / K_ + step_higher_deri * grad_Ja_to_a.transpose() * grad_a_to_t);
        cost_acc += omega * cost_tmp;
      }
    }
    

    // Matrix<double, 3, 3> grad_J_to_d = grad_Jp_to_d * rho_pos_ + grad_Jv_to_d * rho_vel_ + grad_Ja_to_d * rho_acc_;
    // grad_J_to_d *= step_pos * weight;
    Matrix<double, 3, 3> grad_J_to_d = step_pos * grad_Jp_to_d * rho_pos_ + step_higher_deri * (grad_Jv_to_d * rho_vel_ + grad_Ja_to_d * rho_acc_);
    grad_J_to_d *= weight;
    grad_d.head(3) += grad_J_to_d.col(0);
    grad_d.segment(3, 3) += grad_J_to_d.col(1);
    grad_d.tail(3) += grad_J_to_d.col(2);

    double tmp_grad_T = grad_Jp_to_t * rho_pos_ + grad_Jv_to_t * rho_vel_ + grad_Ja_to_t * rho_acc_;
    tmp_grad_T *= weight;
    grad_Tau += tmp_grad_T * exp(Tau);
    // ROS_ERROR_STREAM("itg T grad: " << grad_T);

    // double cost(cost_pos * rho_pos_ + cost_vel * rho_vel_ + cost_acc * rho_acc_);
    // cost *= step_pos * weight;
    double cost(step_pos * cost_pos * rho_pos_ + step_higher_deri * (cost_vel * rho_vel_ + cost_acc * rho_acc_));
    cost *= weight;
    return cost;
  }

  bool collisionCostGrad(const Vector3d& pos, Vector3d& grad, double& cost)
  { 
    double dist(0.0);
    pos_checker_ptr_->evaluateEDTWithGrad(pos, -1, dist, grad);
    // grad.normalize();
    
    double dist_diff = dist_tolerance_ - dist;
    if (dist_diff < 0)
    {
      return false;
    }
    else
    {
      grad *= -3 * dist_diff * dist_diff;
      cost = dist_diff * dist_diff * dist_diff;
      return true;
    }
  }

  /*
  * For penalty of higher derivatives like vel\acc\jerk\...
  f = max((v^2 - v_max^2)^3, 0)
  */
  bool highDerivativeCostGrad(const Vector3d& derivative, 
                              const double &squared_limit, 
                              Vector3d& grad, 
                              double& cost)
  {
    double squared_diff = derivative.squaredNorm() - squared_limit;
    if (squared_diff > 0) 
    {
      grad = 6 * squared_diff * squared_diff * derivative;
      cost = squared_diff * squared_diff * squared_diff;
      return true;
    }
    return false;
  }

  static int _progress(void *instance,
                       const double *x,
                       const double *g,
                       const double fx,
                       const double xnorm,
                       const double gnorm,
                       const double step,
                       int n,
                       int k,
                       int ls)
  {
    return reinterpret_cast<BranchOpt *>(instance)->progress(x, g, fx, xnorm, gnorm, step, n, k, ls);
  }

  int progress(const double *x,
               const double *g,
               const double fx,
               const double xnorm,
               const double gnorm,
               const double step,
               int n,
               int k,
               int ls)
  {
    vector<StatePVA> vis_x;
    vector<Vector3d> knots;

    VectorXd curr_state(9);
    for (int i=0; i<state_size_; i++)
    {
      curr_state[i] = x_[i];
    }
    VectorXd children_Ts(children_size_);
    for (int i=0; i<children_size_; i++)
    {
      children_Ts[i] = x_[i + state_size_];
    }
    double parent_T = x_[children_size_ + state_size_];

    BoundaryCond bc;
    bc.col(0) = state_p_.head(3);
    bc.col(1) = state_p_.segment(3, 3);
    bc.col(2) = state_p_.tail(3);
    bc.col(3) = curr_state.head(3);
    bc.col(4) = curr_state.segment(3, 3);
    bc.col(5) = curr_state.tail(3);
    Piece poly_seg = Piece(bc, parent_T);
    poly_seg.sampleOneSeg(&vis_x);
    knots.push_back(state_p_.head(3));
    knots.push_back(curr_state.head(3));

    int i(0);
    for (const auto& child : state_c_) 
    {
      bc.col(0) = curr_state.head(3);
      bc.col(1) = curr_state.segment(3, 3);
      bc.col(2) = curr_state.tail(3);
      bc.col(3) = child.head(3);
      bc.col(4) = child.segment(3, 3);
      bc.col(5) = child.tail(3);
      Piece poly_seg = Piece(bc, children_Ts[i]);
      poly_seg.sampleOneSeg(&vis_x);
      knots.push_back(child.head(3));
      i++;
    }
    vis_ptr_->visualizeStates(vis_x, BLUE, pos_checker_ptr_->getLocalTime());
    vis_ptr_->visualizeKnots(knots, pos_checker_ptr_->getLocalTime());

    printf("Iteration %d:\n", k);
    printf("  fx = %f, step = %f, xnorm = %f, gnorm = %f\n", fx, step, xnorm, gnorm);
    for (int i=0; i<children_size_; ++i)
      printf("  children %d, T: %f g_T: %f\n", i, exp(x[state_size_ + i]), g[state_size_ + i]);
    printf("  parent, T: %f g_T: %f\n", exp(x[state_size_ + children_size_]), g[state_size_ + children_size_]);
    printf("\n");
    // getchar();

    return 0;
  }
};
