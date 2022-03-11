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
#include "kino_plan/krrt.h"
#include <queue>
#include <unordered_set>

namespace tgk_planner
{
  KRRT::KRRT(const ros::NodeHandle &nh, std::shared_ptr<BVPSolver::IntegratorBVP> &bvp) : sampler_(nh), bvp_(bvp)
  {
  }

  KRRT::~KRRT()
  {
    for (int i = 0; i < tree_node_nums_; i++)
      delete start_tree_[i];
  }

  void KRRT::init(const ros::NodeHandle &nh)
  {
    nh.param("krrt/vel_limit", vel_limit_, -1.0);
    nh.param("krrt/acc_limit", acc_limit_, -1.0);
    nh.param("krrt/jerk_limit", jerk_limit_, -1.0);
    nh.param("krrt/debug_vis", debug_vis_, false);
    nh.param("krrt/rho", rho_, -1.0);
    nh.param("krrt/tree_node_nums", tree_node_nums_, 0);
    nh.param("krrt/radius_cost_between_two_states", radius_cost_between_two_states_, 0.0);
    nh.param("krrt/allow_close_goal", allow_close_goal_, false);
    nh.param("krrt/stop_after_first_traj_found", stop_after_first_traj_found_, false);
    nh.param("krrt/rewire", rewire_, true);
    nh.param("krrt/use_regional_opt", use_regional_opt_, false);
    nh.param("krrt/test_convergency", test_convergency_, false);
    nh.param("krrt/use_deform", use_deform_, true);
    nh.param("krrt/min_deform_duration", min_deform_duration_, 0.8);

    ROS_WARN_STREAM("[krrt] param: vel_limit: " << vel_limit_);
    ROS_WARN_STREAM("[krrt] param: acc_limit: " << acc_limit_);
    ROS_WARN_STREAM("[krrt] param: jerk_limit: " << jerk_limit_);
    ROS_WARN_STREAM("[krrt] param: debug_vis: " << debug_vis_);
    ROS_WARN_STREAM("[krrt] param: rho: " << rho_);
    ROS_WARN_STREAM("[krrt] param: tree_node_nums: " << tree_node_nums_);
    ROS_WARN_STREAM("[krrt] param: radius_cost_between_two_states: " << radius_cost_between_two_states_);
    ROS_WARN_STREAM("[krrt] param: allow_close_goal: " << allow_close_goal_);
    ROS_WARN_STREAM("[krrt] param: stop_after_first_traj_found: " << stop_after_first_traj_found_);
    ROS_WARN_STREAM("[krrt] param: rewire: " << rewire_);
    ROS_WARN_STREAM("[krrt] param: use_regional_opt: " << use_regional_opt_);
    ROS_WARN_STREAM("[krrt] param: use_deform: " << use_deform_);
    ROS_WARN_STREAM("[krrt] param: test_convergency: " << test_convergency_);
    ROS_WARN_STREAM("[krrt] param: min_deform_duration: " << min_deform_duration_);

    valid_start_tree_node_nums_ = 0;

    //pre allocate memory
    start_tree_.resize(tree_node_nums_);
    for (int i = 0; i < tree_node_nums_; i++)
    {
      start_tree_[i] = new RRTNode;
    }
  }

  void KRRT::setPosChecker(const PosChecker::Ptr &checker)
  {
    pos_checker_ptr_ = checker;
    sampler_.setPosChecker(checker);
  }

  void KRRT::setVisualizer(const VisualRviz::Ptr &vis)
  {
    vis_ptr_ = vis;
  }

  // reset() is called every time before plan();
  void KRRT::reset()
  {
    for (int i = 0; i < valid_start_tree_node_nums_; i++)
    {
      start_tree_[i]->parent = nullptr;
      start_tree_[i]->children.clear();
    }
    valid_start_tree_node_nums_ = 0;
  }

  int KRRT::plan(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                 Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                 double search_time, int deform_type, bool use_regional_opt, bool use_deform)
  {
    t_start_ = ros::Time::now();
    deform_type_ = deform_type;
    use_regional_opt_ = use_regional_opt;
    use_deform_ = use_deform;

    if (pos_checker_ptr_->getVoxelState(start_pos) != 0)
    {
      ROS_ERROR("[KRRT]: Start pos collide or out of bound");
      return FAILURE;
    }

    if (!pos_checker_ptr_->validatePosSurround(end_pos))
    {
      Vector3d shift[20] = {Vector3d(-1.0, 0.0, 0.0), Vector3d(0.0, 1.0, 0.0), Vector3d(0.0, -1.0, 0.0),
                            Vector3d(0.0, 0.0, 1.0), Vector3d(0.0, 0.0, -1.0), Vector3d(1.0, 0.0, 0.0),
                            Vector3d(-2.0, 0.0, 0.0), Vector3d(0.0, 2.0, 0.0), Vector3d(0.0, -2.0, 0.0),
                            Vector3d(0.0, 0.0, 2.0), Vector3d(0.0, 0.0, -2.0), Vector3d(2.0, 0.0, 0.0),

                            Vector3d(1.0, 1.0, 1.0), Vector3d(1.0, 1.0, -1.0), Vector3d(1.0, -1.0, 1.0),
                            Vector3d(1.0, -1.0, -1.0), Vector3d(-1.0, 1.0, -1.0), Vector3d(-1.1, 1.0, 1.0),
                            Vector3d(-1.0, -1.0, 1.0), Vector3d(-1.0, -1.0, -1.0)};
      ROS_WARN("[KRRT]: End pos collide or out of bound, search for other safe end");
      int i = 0;
      for (; i < 20; ++i)
      {
        end_pos += shift[i] * 0.2;
        if (pos_checker_ptr_->validatePosSurround(end_pos))
          break;
      }
      if (i == 20)
      {
        ROS_ERROR("found no valid end pos, plan fail");
        return FAILURE;
      }
    }

    if ((start_pos - end_pos).norm() < 1e-3 && (start_vel - end_vel).norm() < 1e-4)
    {
      ROS_ERROR("[KRRT]: start state & end state too close");
      return FAILURE;
    }

    /* construct start and goal nodes */
    start_node_ = start_tree_[1]; //init ptr
    start_node_->x.head(3) = start_pos;
    start_node_->x.segment(3, 3) = start_vel;
    start_node_->x.tail(3) = start_acc;
    start_node_->cost_from_start = 0.0;
    start_node_->tau_from_start = 0.0;
    goal_node_ = start_tree_[0]; //init ptr
    goal_node_->x.head(3) = end_pos;
    if (end_vel.norm() >= vel_limit_)
    {
      end_vel.normalize();
      end_vel = end_vel * vel_limit_;
    }
    goal_node_->x.segment(3, 3) = end_vel;
    goal_node_->x.tail(3) = end_acc;
    goal_node_->cost_from_start = DBL_MAX; //important
    goal_node_->tau_from_start = DBL_MAX;  //important
    valid_start_tree_node_nums_ = 2;       //start and goal already in start_tree_

    /* init sampling space */
    ROS_INFO_STREAM("Kino RRT starts planning");
    vector<Vector3d> p_head, tracks;
    sampler_.getTopo(p_head, tracks);
    vis_ptr_->visualizeTopo(p_head, tracks, pos_checker_ptr_->getLocalTime());

    return rrt(start_node_->x, goal_node_->x, tree_node_nums_, search_time, radius_cost_between_two_states_);
  }

  int KRRT::rrt(const StatePVA &x_init, const StatePVA &x_final, int n, double search_time, double radius)
  {
    ros::Time rrt_start_time = ros::Time::now();
    ros::Time first_goal_found_time, final_goal_found_time;
    double first_general_cost(0.0);
    unordered_set<RRTNodePtr> curr_traj_node_set;

    /* local variables */
    valid_sample_nums_ = 0; //random samples in obs free area
    vector<StatePVA> vis_x;
    vis_x.reserve(3000);
    vector<StatePVA> region_traj_vis_x;
    bool first_time_find_goal(true);
    bool close_goal_found(false);
    double close_dist(DBL_MAX);
    bool goal_found(false);
    traj_list_.clear();
    solution_cost_list_.clear();
    solution_time_list_.clear();

    /* kd tree init */
    kdtree *kd_tree = kd_create(3);
    //Add start and goal nodes to kd tree
    kd_insert3(kd_tree, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);
    kd_insert3(kd_tree, goal_node_->x[0], goal_node_->x[1], goal_node_->x[2], goal_node_);

    //TODO changable radius
    double tau_for_instance = radius * 0.75; //maximum
    double fwd_radius_p = getForwardRadius(tau_for_instance, radius);
    double bcwd_radius_p = getBackwardRadius(tau_for_instance, radius);
    // ROS_INFO_STREAM("bcwd_radius_p: " << bcwd_radius_p);
    // ROS_INFO_STREAM("fwd_radius_p: " << fwd_radius_p);

    /* main loop */
    vector<StatePVA> samples, valid_samples;
    int idx = 0;
    for (idx = 0; (ros::Time::now() - rrt_start_time).toSec() < search_time && valid_start_tree_node_nums_ < n; ++idx)
    {
      /* biased random sampling */
      StatePVA x_rand;
      bool good_sample = sampler_.samplingOnce(idx, x_rand);
      // samples.push_back(x_rand);
      if (!good_sample)
      {
        continue;
      }
      /* make sure that nearby nodes are in different lattices */
      struct kdres *p_nbr_set = kd_nearest_range3(kd_tree, x_rand[0], x_rand[1], x_rand[2], 1);
      if (p_nbr_set == nullptr)
      {
        ROS_ERROR("kd range query error");
        break;
      }
      while (!kd_res_end(p_nbr_set))
      {
        RRTNodePtr curr_node = (RRTNodePtr)kd_res_item_data(p_nbr_set);
        Vector3d dir1(x_rand[3], x_rand[4], x_rand[5]);
        Vector3d dir2(curr_node->x[3], curr_node->x[4], curr_node->x[5]);
        double angle_diff();
        if (dir1.dot(dir2) > 0)
        {
          good_sample = false;
          break;
        }
        kd_res_next(p_nbr_set);
      }
      kd_res_free(p_nbr_set);
      if (!good_sample)
      {
        continue;
      }
      /* make sure that nearby nodes are in different lattices */

      valid_samples.push_back(x_rand);
      ++valid_sample_nums_;

      /* kd_tree bounds search for parent */
      struct kdres *p_bcwd_nbr_set;
      p_bcwd_nbr_set = getBackwardNeighbour(x_rand, kd_tree, radius - tau_for_instance, bcwd_radius_p);
      if (p_bcwd_nbr_set == nullptr)
      {
        ROS_ERROR("bkwd kd range query error");
        break;
      }
      /* choose parent from kd tree range query result*/
      double min_dist(DBL_MAX);
      double tau_from_s(DBL_MAX);
      double cost_from_p(0.0);
      double tau_from_p(0.0);
      RRTNode *x_near(nullptr); //parent
      Piece find_parent_seg;
      while (!kd_res_end(p_bcwd_nbr_set))
      {
        RRTNodePtr curr_node = (RRTNodePtr)kd_res_item_data(p_bcwd_nbr_set);
        if (curr_node == goal_node_)
        {
          // goal node can not be parent of any other node
          kd_res_next(p_bcwd_nbr_set);
          continue;
        }
        if (bvp_->solve(curr_node->x, x_rand, ACC_UNKNOWN))
        {
          CoefficientMat coeff;
          bvp_->getCoeff(coeff);
          Piece seg_from_parent = Piece(bvp_->getTauStar(), coeff);
          // bool connected = checkSegmentConstraints(seg_from_parent);
          bool vel_cons = seg_from_parent.checkMaxVelRate(vel_limit_);
          bool acc_cons = seg_from_parent.checkMaxAccRate(acc_limit_);
          bool jerk_cons = seg_from_parent.checkMaxJerkRate(jerk_limit_);
          pair<Vector3d, Vector3d> collide_pts_one_seg;
          pair<double, double> t_s_e;
          bool need_region_opt(false);
          bool pos_cons = pos_checker_ptr_->checkPolySeg(seg_from_parent, collide_pts_one_seg, t_s_e, need_region_opt);
          bool connected = vel_cons && acc_cons && jerk_cons && pos_cons;
          if (connected)
          {
            if (min_dist > (curr_node->cost_from_start + bvp_->getCostStar()))
            {
              cost_from_p = bvp_->getCostStar();
              tau_from_p = bvp_->getTauStar();
              min_dist = curr_node->cost_from_start + cost_from_p;
              tau_from_s = curr_node->tau_from_start + tau_from_p;
              find_parent_seg = seg_from_parent;
              x_near = curr_node;
              break; //!TODO no find best parent
              // ROS_INFO("one parent found");
            }
          }
        }
        else
        {
          ROS_ERROR("sth. wrong with the bvp solver");
        }
        kd_res_next(p_bcwd_nbr_set); //go to next in kd tree range query result
      }
      kd_res_free(p_bcwd_nbr_set); //reset kd tree range query

      RRTNode *sampled_node(nullptr);
      if (x_near != nullptr)
      {
        /* parent found within radius, then add a node to rrt and kd_tree */
        //sample rejection
        Vector3d calculated_acc = find_parent_seg.getAcc(tau_from_p);
        if (calculated_acc.norm() >= acc_limit_)
          continue;
        x_rand.tail(3) = calculated_acc;
        if (bvp_->solve(x_rand, goal_node_->x, ACC_KNOWN))
        {
          if (min_dist + bvp_->getCostStar() >= goal_node_->cost_from_start)
          {
            // ROS_WARN("parent found but sample rejected");
            continue;
          }
        }
        else
        {
          ROS_ERROR("sth. wrong with the bvp solver");
        }
        /* 1.1 add the randomly sampled node to rrt_tree */
        sampled_node = addTreeNode(x_near, x_rand, find_parent_seg, min_dist, tau_from_s, cost_from_p, tau_from_p);

        /* 1.2 add the randomly sampled node to kd_tree */
        kd_insert3(kd_tree, x_rand[0], x_rand[1], x_rand[2], sampled_node);
      }
      /* end of find parent */

      if (sampled_node == nullptr) // neither w/ or w/o RO found no parent
      {
        continue;
      }

      /* 2.deform */
      RRTNodePtr deforming_node = sampled_node->parent;
      if (goal_found && use_deform_ && deforming_node != start_node_)
      {
        if (deform_type_ == ONE_NODE)
        {
          if (deforming_node->poly_seg.getDuration() > min_deform_duration_) // && deforming_node->children.size() <= 5)
          {
            unitDeformation(deforming_node);
          }
        }
        else if (deform_type_ == TRUNK_NODES)
        {
          vector<RRTNodePtr> deform_node_set;
          while (deforming_node != start_node_)
          {
            deform_node_set.push_back(deforming_node);
            deforming_node = deforming_node->parent;
          }
          if (goal_found && use_deform_)
          {
            for (int i = deform_node_set.size() - 1; i >= 0; i--)
            {
              if (deform_node_set[i]->poly_seg.getDuration() > min_deform_duration_)
              {
                unitDeformation(deform_node_set[i]);
              }
            }
          }
        }
        else if (deform_type_ == BRANCH_NODES)
        {
          RRTNodePtr root_deforming_node(deforming_node);
          while (deforming_node != start_node_)
          {
            root_deforming_node = deforming_node;
            deforming_node = deforming_node->parent;
          }
          if (root_deforming_node != start_node_)
          {
            //breadth first
            queue<RRTNodePtr> q;
            q.push(root_deforming_node);
            while (!q.empty())
            {
              RRTNodePtr deform_node = q.front();
              if (deform_node->poly_seg.getDuration() > min_deform_duration_)
              {
                unitDeformation(deform_node);
              }
              q.pop();
              for (const auto &child : deform_node->children)
              {
                if (!child->children.empty())
                  q.push(child);
              }
            }
          }
        }
        else if (deform_type_ == SMALL_BRANCH_NODES)
        {
          //breadth first
          queue<RRTNodePtr> q;
          q.push(deforming_node);
          while (!q.empty())
          {
            RRTNodePtr deform_node = q.front();
            if (deform_node->poly_seg.getDuration() > min_deform_duration_)
            {
              unitDeformation(deform_node);
            }
            q.pop();
            for (const auto &child : deform_node->children)
            {
              if (!child->children.empty())
                q.push(child);
            }
          }
        }
        else if (deform_type_ == TREE_NODES)
        {
          queue<RRTNodePtr> q;
          for (const auto &node : start_node_->children)
          {
            q.push(node);
          }
          //breadth first
          while (!q.empty())
          {
            RRTNodePtr deform_node = q.front();
            if (deform_node->poly_seg.getDuration() > min_deform_duration_)
            {
              unitDeformation(deform_node);
            }
            q.pop();
            for (const auto &child : deform_node->children)
            {
              if (!child->children.empty())
                q.push(child);
            }
          }
        }
      }

      /* 1. try to connect to goal after a valid tree node found */
      bool new_better_solution(false);
      CoefficientMat coeff;
      bvp_->getCoeff(coeff);
      Piece seg2goal = Piece(bvp_->getTauStar(), coeff);
      // bool connected_to_goal = checkSegmentConstraints(seg2goal);
      bool vel_cons = seg2goal.checkMaxVelRate(vel_limit_);
      bool acc_cons = seg2goal.checkMaxAccRate(acc_limit_);
      bool jerk_cons = seg2goal.checkMaxJerkRate(jerk_limit_);
      pair<Vector3d, Vector3d> collide_pts_one_seg;
      pair<double, double> t_s_e;
      bool need_region_opt(false);
      bool pos_cons = pos_checker_ptr_->checkPolySeg(seg2goal, collide_pts_one_seg, t_s_e, need_region_opt);
      bool connected_to_goal = vel_cons && acc_cons && jerk_cons && pos_cons;
      if (connected_to_goal && goal_node_->cost_from_start > min_dist + bvp_->getCostStar())
      {
        changeNodeParent(goal_node_, sampled_node, seg2goal, bvp_->getCostStar(), bvp_->getTauStar());
        ROS_WARN_STREAM("goal directly connected  curr cost: " << goal_node_->cost_from_start << ", curr dur: " << goal_node_->tau_from_start);
        goal_found = true;
        new_better_solution = true;
      }
      if (new_better_solution)
      {
        /* maintain the set of nodes in curr best traj */
        curr_traj_node_set.clear();
        curr_traj_node_set.clear();
        RRTNodePtr node(goal_node_);
        while (node->parent)
        {
          curr_traj_node_set.insert(node);
          node = node->parent;
        }
        /* maintain the set of nodes in curr best traj */
        if (test_convergency_)
        {
          double curr_general_cost = goal_node_->cost_from_start;
          ros::Time curr_goal_found_time = ros::Time::now();
          double curr_traj_use_time_ = (ros::Time::now() - t_start_).toSec();
          Trajectory traj;
          fillTraj(goal_node_, traj);
          traj_list_.emplace_back(traj);
          solution_cost_list_.emplace_back(curr_general_cost);
          solution_time_list_.emplace_back(curr_traj_use_time_);
        }
        if (first_time_find_goal)
        {
          first_general_cost = goal_node_->cost_from_start;
          first_goal_found_time = ros::Time::now();
          first_traj_use_time_ = (first_goal_found_time - t_start_).toSec();
          first_time_find_goal = false;
          fillTraj(goal_node_, first_traj_);
        }
        if (stop_after_first_traj_found_)
          break; //stop searching after first time find the goal?
      }
      /* end of try to connect to goal */
    } /* end of sample once */
    t_end_ = ros::Time::now();

    vis_x.clear();
    vector<Vector3d> knots;
    sampleWholeTree(start_node_, &vis_x, knots);
    vis_ptr_->visualizeStates(vis_x, TreeTraj, pos_checker_ptr_->getLocalTime());
    vis_ptr_->visualizeKnots(knots, pos_checker_ptr_->getLocalTime());
    // vis_ptr_->visualizeSampledState(samples, pos_checker_ptr_->getLocalTime());
    vis_ptr_->visualizeSampledState(valid_samples, pos_checker_ptr_->getLocalTime());
    // vis_ptr_->visualizePoints(region_traj_vis_x, pos_checker_ptr_->getLocalTime());

    if (goal_found)
    {
      final_traj_use_time_ = (t_end_ - t_start_).toSec();
      fillTraj(goal_node_, traj_);
      /* for traj vis */
      ROS_INFO_STREAM("[KRRT]: total sample times: " << idx);
      ROS_INFO_STREAM("[KRRT]: valid sample times: " << valid_sample_nums_);
      ROS_INFO_STREAM("[KRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
      ROS_INFO("[KRRT]: Time: %.2f s, MaxVel: %.2f m/s, MaxAcc: %.2f m/s^2 , MaxJrk: %.2f m/s^3 \n",
               traj_.getTotalDuration(), traj_.getMaxVelRate(), traj_.getMaxAccRate(), traj_.getMaxJerkRate());
      return SUCCESS;
    }
    else if (allow_close_goal_ && valid_start_tree_node_nums_ > 2)
    {
      ROS_WARN("Not connectting to goal, choose a bypass");
      t_end_ = ros::Time::now();
      chooseBypass(close_goal_node_, start_node_);
      fillTraj(close_goal_node_, traj_);
      /* for traj vis */
      vis_x.clear();
      traj_.sampleWholeTrajectory(&vis_x);
      vis_ptr_->visualizeStates(vis_x, GRAY, pos_checker_ptr_->getLocalTime());

      double final_traj_len(0.0), final_traj_duration(0.0), final_traj_acc_itg(0.0), final_traj_jerk_itg(0.0);
      int final_traj_seg_nums(0);
      evaluateTraj(traj_, final_traj_duration, final_traj_len, final_traj_seg_nums, final_traj_acc_itg, final_traj_jerk_itg);
      final_traj_use_time_ = t_end_.toSec() - t_start_.toSec();
      ROS_INFO_STREAM("[KRRT]: total sample times: " << idx);
      ROS_INFO_STREAM("[KRRT]: valid sample times: " << valid_sample_nums_);
      ROS_INFO_STREAM("[KRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
      ROS_INFO_STREAM("[KRRT]: [front-end final path]: " << endl
                                                         << "    -   seg nums: " << final_traj_seg_nums << endl
                                                         << "    -   time: " << final_traj_use_time_ * 1e3 << " ms" << endl
                                                         << "    -   acc integral: " << final_traj_acc_itg << endl
                                                         << "    -   jerk integral: " << final_traj_jerk_itg << endl
                                                         << "    -   traj duration: " << final_traj_duration << endl
                                                         << "    -   path length: " << final_traj_len << " m" << endl
                                                         << "    -   general cost: " << close_goal_node_->cost_from_start);
      return SUCCESS_CLOSE_GOAL;
    }
    else if (valid_start_tree_node_nums_ == n)
    {
      ROS_ERROR_STREAM("[KRRT]: NOT CONNECTED TO GOAL after " << n << " nodes added to rrt-tree");
      ROS_INFO_STREAM("[KRRT]: total sample times: " << idx);
      ROS_INFO_STREAM("[KRRT]: valid sample times: " << valid_sample_nums_);
      ROS_INFO_STREAM("[KRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
      return FAILURE;
    }
    else if ((ros::Time::now() - rrt_start_time).toSec() >= search_time)
    {
      ROS_ERROR_STREAM("[KRRT]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - rrt_start_time).toSec() << " seconds");
      ROS_INFO_STREAM("[KRRT]: total sample times: " << idx);
      ROS_INFO_STREAM("[KRRT]: valid sample times: " << valid_sample_nums_);
      ROS_INFO_STREAM("[KRRT]: valid tree node nums: " << valid_start_tree_node_nums_);
      return FAILURE;
    }
    else
    {
      return FAILURE;
    }
  }

  double KRRT::getForwardRadius(double tau, double cost)
  {
    MatrixXd G(3, 3);
    G.setZero();
    double tau_2 = tau * tau;
    double tau_3 = tau_2 * tau;
    double tau_4 = tau_3 * tau;
    double tau_5 = tau_4 * tau;
    G(0, 0) = 720.0 / tau_5;
    G(1, 1) = 192.0 / tau_3;
    G(2, 2) = 9.0 / tau;
    G(0, 1) = G(1, 0) = -360.0 / tau_4;
    G(0, 2) = G(2, 0) = 60.0 / tau_3;
    G(1, 2) = G(2, 1) = -36.0 / tau_2;
    G = G * rho_ / (cost - tau) * 3.0;
    // ROS_INFO_STREAM("G: \n" << G);
    Eigen::EigenSolver<MatrixXd> es(G);
    // cout << "The eigenvalues of G are:" << endl << es.eigenvalues() << endl;
    // cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
    double radius_p(0.0);
    for (int i = 0; i < 3; ++i)
    {
      radius_p = max(radius_p, sqrt(1.0 / es.eigenvalues()[i].real()) * fabs(es.eigenvectors().col(i)[i]));
    }
    return radius_p * 1.732;
  }

  double KRRT::getBackwardRadius(double tau, double cost)
  {
    MatrixXd G(3, 3);
    G.setZero();
    double tau_2 = tau * tau;
    double tau_3 = tau_2 * tau;
    double tau_4 = tau_3 * tau;
    double tau_5 = tau_4 * tau;
    G(0, 0) = 720.0 / tau_5;
    G(1, 1) = 192.0 / tau_3;
    G(2, 2) = 9.0 / tau;
    G(0, 1) = G(1, 0) = -360.0 / tau_4;
    G(0, 2) = G(2, 0) = 60.0 / tau_3;
    G(1, 2) = G(2, 1) = -36.0 / tau_2;
    G = G * rho_ / (cost - tau) * 3.0;
    // ROS_INFO_STREAM("G: \n" << G);
    Eigen::EigenSolver<MatrixXd> es(G);
    // cout << "The eigenvalues of G are:" << endl << es.eigenvalues() << endl;
    // cout << "The matrix of eigenvectors, V, is:" << endl << es.eigenvectors() << endl << endl;
    double radius_p(0.0);
    for (int i = 0; i < 3; ++i)
    {
      radius_p = max(radius_p, sqrt(1.0 / es.eigenvalues()[i].real()) * fabs(es.eigenvectors().col(i)[i]));
    }
    return radius_p * 1.732;
  }

  struct kdres *KRRT::getForwardNeighbour(const StatePVA &x0, struct kdtree *kd_tree, double tau, double radius_p)
  {
    double half_tau_square = tau * tau / 2;
    StatePVA x_ba_tau;
    x_ba_tau[0] = x0[0] + x0[3] * tau + x0[6] * half_tau_square;
    x_ba_tau[1] = x0[1] + x0[4] * tau + x0[7] * half_tau_square;
    x_ba_tau[2] = x0[2] + x0[5] * tau + x0[8] * half_tau_square;
    // x_ba_tau[3] = x0[3] + tau*x0[6];
    // x_ba_tau[4] = x0[4] + tau*x0[7];
    // x_ba_tau[5] = x0[5] + tau*x0[8];
    // x_ba_tau[6] = x0[6];
    // x_ba_tau[7] = x0[7];
    // x_ba_tau[8] = x0[8];

    if (debug_vis_)
    {
      vis_ptr_->visualizeReachPos(FORWARD_REACHABLE_POS, x_ba_tau.head(3), 2 * radius_p, pos_checker_ptr_->getLocalTime());
      // vis_ptr_->visualizeReachVel(1, x_ba_tau.head(3) + x_ba_tau.tail(3), 2 * radius_v, pos_checker_ptr_->getLocalTime());
      // getchar();
    }
    return kd_nearest_range3(kd_tree, x_ba_tau[0], x_ba_tau[1], x_ba_tau[2], radius_p);
  }

  struct kdres *KRRT::getBackwardNeighbour(const StatePVA &x1, struct kdtree *kd_tree, double tau, double radius_p)
  {
    StatePVA expNegATau_x1;
    double half_tau_square = tau * tau / 2;
    expNegATau_x1[0] = x1[0] - x1[3] * tau + x1[6] * half_tau_square;
    expNegATau_x1[1] = x1[1] - x1[4] * tau + x1[7] * half_tau_square;
    expNegATau_x1[2] = x1[2] - x1[5] * tau + x1[8] * half_tau_square;
    // expNegATau_x1[3] = x1[3] - x1[6]*tau;
    // expNegATau_x1[4] = x1[4] - x1[7]*tau;
    // expNegATau_x1[5] = x1[5] - x1[8]*tau;
    // expNegATau_x1[6] = x1[6];
    // expNegATau_x1[7] = x1[7];
    // expNegATau_x1[8] = x1[8];

    if (debug_vis_)
    {
      vis_ptr_->visualizeReachPos(BACKWARD_REACHABLE_POS, expNegATau_x1.head(3), 2 * radius_p, pos_checker_ptr_->getLocalTime());
      // vis_ptr_->visualizeReachVel(1, expNegATau_x1.head(3) + expNegATau_x1.tail(3), 2 * radius_v, pos_checker_ptr_->getLocalTime());
      // getchar();
    }
    return kd_nearest_range3(kd_tree, expNegATau_x1[0], expNegATau_x1[1], expNegATau_x1[2], radius_p);
  }

  inline RRTNodePtr KRRT::addTreeNode(RRTNodePtr &parent, const StatePVA &state, const Piece &piece,
                                      const double &cost_from_start, const double &tau_from_start,
                                      const double &cost_from_parent, const double &tau_from_parent)
  {
    RRTNodePtr new_node_ptr = start_tree_[valid_start_tree_node_nums_++];
    new_node_ptr->parent = parent;
    parent->children.push_back(new_node_ptr);
    new_node_ptr->x = state;
    new_node_ptr->poly_seg = piece;
    new_node_ptr->cost_from_start = cost_from_start;
    new_node_ptr->tau_from_start = tau_from_start;
    new_node_ptr->cost_from_parent = cost_from_parent;
    new_node_ptr->tau_from_parent = tau_from_parent;
    return new_node_ptr;
  }

  inline RRTNodePtr KRRT::addTreeNode(RRTNodePtr &parent, const StatePVA &state, const Piece &piece,
                                      const double &cost_from_parent, const double &tau_from_parent)
  {
    RRTNodePtr new_node_ptr = start_tree_[valid_start_tree_node_nums_++];
    new_node_ptr->parent = parent;
    parent->children.push_back(new_node_ptr);
    new_node_ptr->x = state;
    new_node_ptr->poly_seg = piece;
    new_node_ptr->cost_from_start = parent->cost_from_start + cost_from_parent;
    new_node_ptr->tau_from_start = parent->tau_from_start + tau_from_parent;
    new_node_ptr->cost_from_parent = cost_from_parent;
    new_node_ptr->tau_from_parent = tau_from_parent;
    return new_node_ptr;
  }

  inline void KRRT::changeNodeParent(RRTNodePtr &node, RRTNodePtr &parent, const Piece &piece,
                                     const double &cost_from_parent, const double &tau_from_parent)
  {
    if (node->parent)
      node->parent->children.remove(node); //DON'T FORGET THIS, remove it form its parent's children list
    node->parent = parent;
    node->cost_from_parent = cost_from_parent;
    node->tau_from_parent = tau_from_parent;
    node->cost_from_start = parent->cost_from_start + cost_from_parent;
    node->tau_from_start = parent->tau_from_start + tau_from_parent;
    node->poly_seg = piece;
    parent->children.push_back(node);

    // for all its descedants, change the cost_from_start and tau_from_start;
    RRTNode *descendant(node);
    std::queue<RRTNode *> Q;
    Q.push(descendant);
    while (!Q.empty())
    {
      descendant = Q.front();
      Q.pop();
      for (const auto &leafptr : descendant->children)
      {
        leafptr->cost_from_start = leafptr->cost_from_parent + descendant->cost_from_start;
        leafptr->tau_from_start = leafptr->tau_from_parent + descendant->tau_from_start;
        Q.push(leafptr);
      }
    }
  }

  inline bool KRRT::regionalOpt(const Piece &oringin_seg, const pair<Vector3d, Vector3d> &collide_pts_one_seg, const pair<double, double> &t_s_e)
  {
    int split_seg_num = 2;
    Trajectory pre_regional_traj;
    pre_regional_traj.reserve(split_seg_num);
    CoefficientMat coeff = oringin_seg.getCoeffMat();
    double duration = oringin_seg.getDuration() / split_seg_num;
    pre_regional_traj.emplace_back(Piece(duration, coeff));
    for (int i = 1; i < split_seg_num; ++i)
    {
      oringin_seg.cutPiece(oringin_seg, duration * i, coeff);
      pre_regional_traj.emplace_back(Piece(duration, coeff));
    }

    if (searcher_->AstarSearch(pos_checker_ptr_->getResolution(), collide_pts_one_seg.first, collide_pts_one_seg.second))
    {
      vector<Eigen::Vector3d> grid_path = searcher_->getPath();
      // vis_ptr_->visualizeKnots(grid_path, pos_checker_ptr_->getLocalTime());
      std::vector<pair<int, int>> seg_num_obs_size; //first: # of segment in a traj; second: number of attract_pt in this segment.
      std::vector<double> t_s, t_e;                 // each attract_pt's timestamp of start and end in its corresponding segment. Size should be the same as the sum of seg_num_obs_size.second.
      std::vector<Eigen::Vector3d> attract_pts;
      pos_checker_ptr_->getRegionalAttractPts(pre_regional_traj, grid_path, t_s_e, seg_num_obs_size, attract_pts, t_s, t_e);
      if (!optimizer_ptr_->initialize(pre_regional_traj, TrajOptimizer::SMOOTH_HOMO_OBS))
      {
        return false;
      }
      return (optimizer_ptr_->solveRegionalOpt(seg_num_obs_size, attract_pts, t_s, t_e));
    }
    else
    {
      // ROS_WARN_STREAM("a star search fail, from: " << collide_pts_one_seg.first.transpose() << " to " << collide_pts_one_seg.second.transpose());
      return false;
    }
  }

  inline void KRRT::unitDeformation(const RRTNodePtr &deforming_node)
  {
    // vector<StatePVA> vis_x;
    // vis_x.clear();
    // vector<Vector3d> knots;
    // sampleWholeTree(start_node_, &vis_x, knots);
    // vis_ptr_->visualizeStates(vis_x, TreeTraj, pos_checker_ptr_->getLocalTime());

    // vis_x.clear();
    // vector<Vector3d> knots1;
    // sampleDeformedTrunk(deforming_node, &vis_x, knots1);
    // vis_ptr_->visualizeStates(vis_x, ORANGE, pos_checker_ptr_->getLocalTime());
    // vis_ptr_->visualizeKnots(knots1, pos_checker_ptr_->getLocalTime());
    // getchar();

    // ROS_WARN("deform start");
    ros::Time deform_s_t = ros::Time::now();
    VectorXd node_state = deforming_node->x;
    VectorXd parent_state = deforming_node->parent->x;
    double parent_T = deforming_node->poly_seg.getDuration();
    int self_descendant_num_plus_one(1);
    vector<VectorXd> children_states;
    vector<double> children_Ts;
    vector<int> children_descendant_num;
    for (const RRTNodePtr &child : deforming_node->children)
    {
      children_states.push_back(child->x);
      children_Ts.push_back(child->poly_seg.getDuration());
      int descendant_num_plus_one = child->countDescendant() + 1;
      children_descendant_num.push_back(descendant_num_plus_one);
      self_descendant_num_plus_one += descendant_num_plus_one;
    }
    // cout << "before deform parent_T: " << parent_T << endl;
    bool deform_result = trunk_opt_ptr_->optimizeBranch(node_state,
                                                        parent_state, parent_T, self_descendant_num_plus_one,
                                                        children_states, children_Ts, children_descendant_num);
    // cout << "after deform parent_T: " << parent_T << endl;
    if (deform_result)
    {
      double before_deform_curr_general_cost = goal_node_->cost_from_start;
      deforming_node->x = node_state;
      BoundaryCond bc;
      bc.col(0) = deforming_node->parent->x.head(3);
      bc.col(1) = deforming_node->parent->x.segment(3, 3);
      bc.col(2) = deforming_node->parent->x.tail(3);
      bc.col(3) = deforming_node->x.head(3);
      bc.col(4) = deforming_node->x.segment(3, 3);
      bc.col(5) = deforming_node->x.tail(3);
      deforming_node->poly_seg = Piece(bc, parent_T);
      // ROS_INFO_STREAM("max vel: " << deforming_node->poly_seg.getMaxVelRate() << ", max acc: " << deforming_node->poly_seg.getMaxAccRate());
      deforming_node->cost_from_parent = bvp_->calCostAccKnown(deforming_node->parent->x, deforming_node->x, parent_T);
      // deforming_node->cost_from_parent = deforming_node->poly_seg.calCost(rho_);
      deforming_node->tau_from_parent = parent_T;
      deforming_node->cost_from_start = deforming_node->parent->cost_from_start + deforming_node->cost_from_parent;
      deforming_node->tau_from_start = deforming_node->parent->tau_from_start + parent_T;
      int i(0);
      for (const RRTNodePtr &child : deforming_node->children)
      {
        bc.col(0) = deforming_node->x.head(3);
        bc.col(1) = deforming_node->x.segment(3, 3);
        bc.col(2) = deforming_node->x.tail(3);
        bc.col(3) = child->x.head(3);
        bc.col(4) = child->x.segment(3, 3);
        bc.col(5) = child->x.tail(3);
        child->poly_seg = Piece(bc, children_Ts[i]);
        // ROS_INFO_STREAM("max vel: " << child->poly_seg.getMaxVelRate() << ", max acc: " << child->poly_seg.getMaxAccRate());
        child->cost_from_parent = bvp_->calCostAccKnown(child->parent->x, child->x, children_Ts[i]);
        // child->cost_from_parent = child->poly_seg.calCost(rho_);
        child->tau_from_parent = children_Ts[i];
        child->cost_from_start = child->parent->cost_from_start + child->cost_from_parent;
        child->tau_from_start = child->parent->tau_from_start + children_Ts[i];
        i++;
        /* change cost_from_start and tau_from_start of nodes in each sub-branch */
        // breadth first search
        queue<RRTNodePtr> q;
        for (const RRTNodePtr &grand_child : child->children)
        {
          q.push(grand_child);
        }
        while (!q.empty())
        {
          RRTNodePtr node = q.front();
          q.pop();
          node->cost_from_start = node->parent->cost_from_start + node->cost_from_parent;
          node->tau_from_start = node->parent->tau_from_start + node->tau_from_parent;
          for (const RRTNodePtr &grand_grand_child : node->children)
          {
            q.push(grand_grand_child);
          }
        }
        /* change cost_from_start and tau_from_start of nodes in each sub-branch */
      }

      double curr_general_cost = goal_node_->cost_from_start;
      if (test_convergency_ && curr_general_cost < before_deform_curr_general_cost - 1e-5)
      {
        ros::Time curr_goal_found_time = ros::Time::now();
        double curr_traj_use_time_ = (ros::Time::now() - t_start_).toSec();
        std::vector<double> durs;
        std::vector<CoefficientMat> coeffMats;
        RRTNodePtr node = goal_node_;
        while (node->parent)
        {
          durs.push_back(node->poly_seg.getDuration());
          coeffMats.push_back(node->poly_seg.getCoeffMat());
          node = node->parent;
        }
        std::reverse(std::begin(durs), std::end(durs));
        std::reverse(std::begin(coeffMats), std::end(coeffMats));
        traj_list_.emplace_back(durs, coeffMats);
        solution_cost_list_.emplace_back(curr_general_cost);
        solution_time_list_.emplace_back(curr_traj_use_time_);
        ROS_WARN_STREAM("deform improved          curr cost: " << curr_general_cost << ", curr dur: " << goal_node_->tau_from_start);
      }
      ros::Time deform_e_t = ros::Time::now();
      // ROS_WARN_STREAM(deforming_node->children.size() + 1 << " segs, deforming success, takes "
      // << (deform_e_t - deform_s_t).toSec()*1000 << " ms, cost change after deform: " << curr_general_cost - before_deform_curr_general_cost);
    }
    else
    {
      // ros::Time deform_e_t = ros::Time::now();
      // ROS_ERROR_STREAM(deforming_node->children.size()+1 << " segs, deforming faild, takes " << (deform_e_t - deform_s_t).toSec()*1000 << " ms");
    }

    // vis_x.clear();
    // sampleDeformedTrunk(deforming_node, &vis_x, knots1);
    // vis_ptr_->visualizeStates(vis_x, GREEN, pos_checker_ptr_->getLocalTime());
    // vis_ptr_->visualizeKnots(knots1, pos_checker_ptr_->getLocalTime());
    // getchar();
  }

  inline bool KRRT::stOpt(const Piece &oringin_seg, const RRTNodePtr &parent, Trajectory &opted_two_seg)
  {
    // vector<StatePVA> vis_x;
    // oringin_seg.sampleOneSeg(&vis_x);
    // vis_ptr_->visualizeStates(vis_x, BLUE, pos_checker_ptr_->getLocalTime());
    // getchar();

    double T = oringin_seg.getDuration();
    double init_T = T / 2.0;
    VectorXd node_state(9);
    node_state.head(3) = oringin_seg.getPos(init_T);
    node_state.segment(3, 3) = oringin_seg.getVel(init_T);
    node_state.tail(3) = oringin_seg.getAcc(init_T);

    VectorXd parent_state = parent->x;
    double parent_T = init_T;

    int self_descendant_num_plus_one(2);
    vector<VectorXd> children_states;
    vector<double> children_Ts;
    vector<int> children_descendant_num;
    VectorXd child_state(9);
    child_state.head(3) = oringin_seg.getPos(T);
    child_state.segment(3, 3) = oringin_seg.getVel(T);
    child_state.tail(3) = oringin_seg.getAcc(T);

    children_states.push_back(child_state);
    children_Ts.push_back(init_T);
    int descendant_num_plus_one = 1;
    children_descendant_num.push_back(descendant_num_plus_one);
    bool deform_result = trunk_opt_ptr_->optimizeBranch(node_state,
                                                        parent_state, parent_T, self_descendant_num_plus_one,
                                                        children_states, children_Ts, children_descendant_num);
    if (deform_result)
    {
      std::vector<double> durs;
      durs.push_back(parent_T);
      durs.push_back(children_Ts[0]);
      std::vector<CoefficientMat> coeffMats;
      BoundaryCond bc1;
      bc1.col(0) = parent->x.head(3);
      bc1.col(1) = parent->x.segment(3, 3);
      bc1.col(2) = parent->x.tail(3);
      bc1.col(3) = node_state.head(3);
      bc1.col(4) = node_state.segment(3, 3);
      bc1.col(5) = node_state.tail(3);
      Piece p1(bc1, parent_T);
      BoundaryCond bc2;
      bc2.col(0) = node_state.head(3);
      bc2.col(1) = node_state.segment(3, 3);
      bc2.col(2) = node_state.tail(3);
      bc2.col(3) = child_state.head(3);
      bc2.col(4) = child_state.segment(3, 3);
      bc2.col(5) = child_state.tail(3);
      Piece p2(bc2, children_Ts[0]);
      coeffMats.push_back(p1.getCoeffMat());
      coeffMats.push_back(p2.getCoeffMat());
      opted_two_seg = Trajectory(durs, coeffMats);

      // vector<StatePVA> vis_x;
      // p1.sampleOneSeg(&vis_x);
      // p2.sampleOneSeg(&vis_x);
      // vis_ptr_->visualizeStates(vis_x, RED, pos_checker_ptr_->getLocalTime());
      // getchar();
    }
    return deform_result;
  }

  inline bool KRRT::stOpt(Trajectory &opted_two_seg)
  {
    vector<StatePVA> vis_x;
    opted_two_seg.sampleWholeTrajectory(&vis_x);
    vis_ptr_->visualizeStates(vis_x, GREEN, pos_checker_ptr_->getLocalTime());
    getchar();

    double parent_T = opted_two_seg[0].getDuration();
    VectorXd node_state(9);
    node_state.head(3) = opted_two_seg[0].getPos(parent_T);
    node_state.segment(3, 3) = opted_two_seg[0].getVel(parent_T);
    node_state.tail(3) = opted_two_seg[0].getAcc(parent_T);

    VectorXd parent_state(9);
    parent_state.head(3) = opted_two_seg[0].getPos(0);
    parent_state.segment(3, 3) = opted_two_seg[0].getVel(0);
    parent_state.tail(3) = opted_two_seg[0].getAcc(0);

    double child_T = opted_two_seg[1].getDuration();
    int self_descendant_num_plus_one(2);
    vector<VectorXd> children_states;
    vector<double> children_Ts;
    vector<int> children_descendant_num;
    VectorXd child_state(9);
    child_state.head(3) = opted_two_seg[1].getPos(child_T);
    child_state.segment(3, 3) = opted_two_seg[1].getVel(child_T);
    child_state.tail(3) = opted_two_seg[1].getAcc(child_T);

    children_states.push_back(child_state);
    children_Ts.push_back(child_T);
    int descendant_num_plus_one = 1;
    children_descendant_num.push_back(descendant_num_plus_one);
    bool deform_result = trunk_opt_ptr_->optimizeBranch(node_state,
                                                        parent_state, parent_T, self_descendant_num_plus_one,
                                                        children_states, children_Ts, children_descendant_num);
    if (deform_result)
    {
      std::vector<double> durs;
      durs.push_back(parent_T);
      durs.push_back(children_Ts[0]);
      std::vector<CoefficientMat> coeffMats;
      BoundaryCond bc1;
      bc1.col(0) = parent_state.head(3);
      bc1.col(1) = parent_state.segment(3, 3);
      bc1.col(2) = parent_state.tail(3);
      bc1.col(3) = node_state.head(3);
      bc1.col(4) = node_state.segment(3, 3);
      bc1.col(5) = node_state.tail(3);
      Piece p1(bc1, parent_T);
      BoundaryCond bc2;
      bc2.col(0) = node_state.head(3);
      bc2.col(1) = node_state.segment(3, 3);
      bc2.col(2) = node_state.tail(3);
      bc2.col(3) = child_state.head(3);
      bc2.col(4) = child_state.segment(3, 3);
      bc2.col(5) = child_state.tail(3);
      Piece p2(bc2, children_Ts[0]);
      coeffMats.push_back(p1.getCoeffMat());
      coeffMats.push_back(p2.getCoeffMat());
      opted_two_seg = Trajectory(durs, coeffMats);

      vector<StatePVA> vis_x;
      p1.sampleOneSeg(&vis_x);
      p2.sampleOneSeg(&vis_x);
      vis_ptr_->visualizeStates(vis_x, RED, pos_checker_ptr_->getLocalTime());
      getchar();
    }
    return deform_result;
  }

  inline void KRRT::sampleDeformedTrunk(const RRTNodePtr &node, vector<StatePVA> *vis_x, vector<Vector3d> &knots)
  {
    if (node == nullptr || node->parent == nullptr)
      return;
    node->poly_seg.sampleOneSeg(vis_x);
    knots.push_back(node->parent->x.head(3));
    for (const auto &leaf : node->children)
    {
      leaf->poly_seg.sampleOneSeg(vis_x);
      knots.push_back(leaf->x.head(3));
    }
    knots.push_back(node->x.head(3));
  }

  inline double KRRT::dist(const StatePVA &x0, const StatePVA &x1)
  {
    Vector3d p_diff(x0.head(3) - x1.head(3));
    return p_diff.norm();
  }

  inline void KRRT::sampleWholeTree(const RRTNodePtr &root, vector<StatePVA> *vis_x, vector<Vector3d> &knots)
  {
    if (root == nullptr)
      return;
    //whatever dfs or bfs
    RRTNode *node = root;
    std::queue<RRTNode *> Q;
    Q.push(node);
    while (!Q.empty())
    {
      node = Q.front();
      Q.pop();
      for (const auto &leafptr : node->children)
      {
        leafptr->poly_seg.sampleOneSeg(vis_x);
        knots.push_back(leafptr->x.head(3));
        Q.push(leafptr);
      }
    }
  }

  void KRRT::fillTraj(const RRTNodePtr &goal_leaf, Trajectory &traj)
  {
    std::vector<double> durs;
    std::vector<CoefficientMat> coeffMats;
    RRTNodePtr node = goal_leaf;
    while (node->parent)
    {
      durs.push_back(node->poly_seg.getDuration());
      coeffMats.push_back(node->poly_seg.getCoeffMat());
      node = node->parent;
    }
    std::reverse(std::begin(durs), std::end(durs));
    std::reverse(std::begin(coeffMats), std::end(coeffMats));
    traj = Trajectory(durs, coeffMats);
  }

  inline bool KRRT::checkSegmentConstraints(const Piece &seg)
  {
    if (!seg.checkMaxVelRate(vel_limit_))
    {
      // ROS_WARN("vel constraints violate!");
      return false;
    }
    if (!seg.checkMaxAccRate(acc_limit_))
    {
      // ROS_WARN("acc constraints violate!");
      return false;
    }
    if (!seg.checkMaxJerkRate(jerk_limit_))
    {
      // ROS_WARN("jerk constraints violate!");
      return false;
    }
    if (!pos_checker_ptr_->checkPolySeg(seg))
    {
      // ROS_WARN("pos constraints violate!");
      return false;
    }
    return true;
  }

  inline bool KRRT::getTraversalLines(const Piece &seg, vector<pair<Vector3d, Vector3d>> &traversal_lines)
  {
    bool res = pos_checker_ptr_->checkPolySeg(seg, traversal_lines);
    return res;
  }

  double KRRT::evaluateTraj(const Trajectory &traj, double &traj_duration, double &traj_length, int &seg_nums,
                            double &acc_integral, double &jerk_integral)
  {
    traj_length = 0.0;
    traj_duration = 0.0;
    seg_nums = traj.getPieceNum();
    acc_integral = 0.0;
    jerk_integral = 0.0;

    double d_t = 0.02;
    for (int i = 0; i < seg_nums; ++i)
    {
      double tau = traj[i].getDuration();
      traj_duration += tau;
      for (double t = 0.0; t < tau; t += d_t)
      {
        Eigen::Vector3d vel, acc, jerk;
        vel = traj[i].getVel(t);
        acc = traj[i].getAcc(t);
        jerk = traj[i].getJerk(t);
        traj_length += vel.norm() * d_t;
        acc_integral += acc.dot(acc) * d_t;
        jerk_integral += jerk.dot(jerk) * d_t;
      }
    }
    double c[seg_nums];
    return traj.calCost(rho_, c);
  }

  void KRRT::chooseBypass(RRTNodePtr &goal_leaf, const RRTNodePtr &tree_start_node)
  {
    goal_leaf = start_tree_[valid_start_tree_node_nums_ - 1];
    double close_dist(DBL_MAX);
    for (int i = 2; i < valid_start_tree_node_nums_; ++i)
    {
      double dist_to_goal = dist(start_tree_[i]->x, goal_node_->x);
      if (dist_to_goal < close_dist)
      {
        close_dist = dist_to_goal;
        close_goal_node_ = start_tree_[i];
      }
    }
  }

} //namespace tgk_planner