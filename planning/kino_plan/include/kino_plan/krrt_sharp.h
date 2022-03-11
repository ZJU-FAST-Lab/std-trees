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
#ifndef _KRRT_SHARP_H_
#define _KRRT_SHARP_H_

#include "node_utils.h"
#include "kdtree.h"
#include "visualization_utils/visualization_utils.h"
#include "occ_grid/pos_checker.h"
#include "poly_traj_utils/traj_utils.hpp"
#include "bvp_solver.h"
#include "sampler/bias_sampler.h"
#include "poly_opt/traj_optimizer.h"
#include "poly_opt/nonsmooth_trunk_opt.h"
// #include "poly_opt/trunk_opt.h"
#include "r3_plan/a_star_search.h"

#include <vector>
#include <stack>

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector2i;
using Eigen::Vector3d;
using Eigen::Vector3i;
using Eigen::VectorXd;
using std::list;
using std::pair;
using std::stack;
using std::vector;

namespace tgk_planner
{

  class KRRTSHARP
  {
  public:
    KRRTSHARP();
    KRRTSHARP(const ros::NodeHandle &nh, std::shared_ptr<BVPSolver::IntegratorBVP> &bvp);
    ~KRRTSHARP();

    // api
    void reset();
    void init(const ros::NodeHandle &nh);
    void setPosChecker(const PosChecker::Ptr &checker);
    void setVisualizer(const VisualRviz::Ptr &vis);
    void setRegionalOptimizer(const TrajOptimizer::Ptr &optimizer_)
    {
      optimizer_ptr_ = optimizer_;
    };
    void setSearcher(const std::shared_ptr<AstarPathFinder> &searcher)
    {
      searcher_ = searcher;
    }
    void setTrunkOptimizer(const std::shared_ptr<BranchOpt> &ptr)
    {
      trunk_opt_ptr_ = ptr;
    };
    int plan(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
             Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
             double search_time, int deform_type, bool use_regional_opt, bool use_deform = false);
    void getTraj(Trajectory &traj)
    {
      traj = traj_;
    };
    void getFirstTraj(Trajectory &traj)
    {
      traj = first_traj_;
    };
    double getFirstTrajTimeUsage()
    {
      return first_traj_use_time_;
    };
    double getFinalTrajTimeUsage()
    {
      return final_traj_use_time_;
    };
    int getSampleNum()
    {
      return valid_sample_nums_;
    };
    int getTreeNodeNum()
    {
      return valid_start_tree_node_nums_;
    };
    void getConvergenceInfo(vector<Trajectory> &traj_list, vector<double> &solution_cost_list, vector<double> &solution_time_list)
    {
      traj_list = traj_list_;
      solution_time_list = solution_time_list_;
      solution_cost_list = solution_cost_list_;
    };

    // evaluation
    double evaluateTraj(const Trajectory &traj, double &traj_duration, double &traj_length, int &seg_nums, double &acc_integral, double &jerk_integral);

    // bias_sampler
    BiasSampler sampler_;

    enum
    {
      FAILURE = 0,
      SUCCESS = 1,
      SUCCESS_CLOSE_GOAL = 2
    };
    typedef shared_ptr<KRRTSHARP> KRRTSHARPPtr;

  private:
    int rrtStar(const StatePVA &x_init, const StatePVA &x_final, int n, double search_time, double radius, const bool rewire);
    double dist(const StatePVA &x0, const StatePVA &x1);
    void fillTraj(const RRTNodePtr &goal_leaf, Trajectory &traj);
    void chooseBypass(RRTNodePtr &goal_leaf, const RRTNodePtr &tree_start_node);
    RRTNodePtr addTreeNode(RRTNodePtr &parent, const StatePVA &state, const Piece &piece,
                           const double &cost_from_start, const double &tau_from_start,
                           const double &cost_from_parent, const double &tau_from_parent);
    RRTNodePtr addTreeNode(RRTNodePtr &parent, const StatePVA &state, const Piece &piece,
                           const double &cost_from_parent, const double &tau_from_parent);
    void changeNodeParent(RRTNodePtr &node, RRTNodePtr &parent, const Piece &piece,
                          const double &cost_from_parent, const double &tau_from_parent);
    void changeNodeParent(RRTNodePtr &node, RRTNodePtr &parent, const Piece &piece,
                          const double &cost_from_parent, const double &tau_from_parent,
                          priority_queue<RRTNodePtr, std::vector<RRTNodePtr>, RRTNodeComparator> &queue);
    bool regionalOpt(const Piece &oringin_seg, const pair<Vector3d, Vector3d> &collide_pts_one_seg, const pair<double, double> &t_s_e);

    // vis
    bool debug_vis_;
    VisualRviz::Ptr vis_ptr_;
    void sampleWholeTree(const RRTNodePtr &root, vector<StatePVA> *vis_x, vector<Vector3d> &knots);

    RRTNodePtrVector start_tree_; //pre allocated in Constructor
    Trajectory traj_;
    Trajectory first_traj_; //initialized when first path found
    RRTNodePtr start_node_, goal_node_, close_goal_node_;
    int valid_start_tree_node_nums_, valid_sample_nums_;
    double final_traj_use_time_, first_traj_use_time_;
    bool test_convergency_;
    vector<Trajectory> traj_list_;
    vector<double> solution_cost_list_;
    vector<double> solution_time_list_;
    ros::Time t_start_, t_end_;

    // radius for for/backward search
    double getForwardRadius(double tau, double cost);
    double getBackwardRadius(double tau, double cost);
    struct kdres *getForwardNeighbour(const StatePVA &x1, struct kdtree *kd_tree, double tau, double radius_p);
    struct kdres *getBackwardNeighbour(const StatePVA &x1, struct kdtree *kd_tree, double tau, double radius_p);

    // nodehandle params
    double radius_cost_between_two_states_;
    double rho_;
    double v_mag_sample_;
    double vel_limit_, acc_limit_, jerk_limit_;
    bool allow_close_goal_, stop_after_first_traj_found_, rewire_, use_regional_opt_, use_deform_;
    int deform_type_;
    double search_time_;
    int tree_node_nums_;
    double min_deform_duration_;

    // environment
    PosChecker::Ptr pos_checker_ptr_;
    bool checkSegmentConstraints(const Piece &seg);
    bool getTraversalLines(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines);

    // bvp_solver
    std::shared_ptr<BVPSolver::IntegratorBVP> bvp_;

    // // bias_sampler
    // BiasSampler sampler_;

    // regional optimizer
    TrajOptimizer::Ptr optimizer_ptr_;
    std::shared_ptr<AstarPathFinder> searcher_;
    std::shared_ptr<BranchOpt> trunk_opt_ptr_;
    bool stOpt(const Piece &oringin_seg, const RRTNodePtr &parent, Trajectory &opted_two_seg);
    bool stOpt(Trajectory &opted_two_seg);
    void sampleDeformedTrunk(const RRTNodePtr &node, vector<StatePVA> *vis_x, vector<Vector3d> &knots);
    void unitDeformation(const RRTNodePtr &deforming_node);
  };

} // namespace tgk_planner

#endif //_KRRT_SHARP_H_
