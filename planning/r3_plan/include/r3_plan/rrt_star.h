/*  
The MIT License

Copyright (c) 2020-2022 Hongkai Ye

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#ifndef RRT_STAR_H
#define RRT_STAR_H

#include "occ_grid/pos_checker.h"
#include "sampler/bias_sampler.h"
#include <ros/ros.h>
#include "node.h"
#include "kdtree.h"

namespace path_plan
{
  class RRTStar
  {
  public:
    RRTStar();
    RRTStar(const ros::NodeHandle &nh, const tgk_planner::PosChecker::Ptr &mapPtr) : nh_(nh), pos_checker_ptr_(mapPtr), sampler_(nh)
    {
      sampler_.setPosChecker(pos_checker_ptr_);

      nh_.param("rrt/steer_length", steer_length_, 0.0);
      nh_.param("rrt/search_radius", search_radius_, 0.0);
      nh_.param("rrt/search_time", search_time_, 0.0);
      nh_.param("rrt/max_tree_node_nums", max_tree_node_nums_, 0);
      ROS_WARN_STREAM("[rrt] param: steer_length: " << steer_length_);
      ROS_WARN_STREAM("[rrt] param: search_radius: " << search_radius_);
      ROS_WARN_STREAM("[rrt] param: search_time: " << search_time_);
      ROS_WARN_STREAM("[rrt] param: max_tree_node_nums: " << max_tree_node_nums_);

      valid_tree_node_nums_ = 0;
      nodes_pool_.resize(max_tree_node_nums_);
      for (int i = 0; i < max_tree_node_nums_; ++i)
      {
        nodes_pool_[i] = new TreeNode;
      }
    }

    bool plan(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      reset();
      if (!pos_checker_ptr_->validatePosSurround(s))
      {
        ROS_ERROR("[rrt]: Start pos collide or out of bound");
        return false;
      }
      if (!pos_checker_ptr_->validatePosSurround(g))
      {
        ROS_ERROR("[rrt]: Goal pos collide or out of bound");
        return false;
      }
      /* construct start and goal nodes */
      start_node_ = nodes_pool_[1];
      start_node_->x = s;
      start_node_->cost_from_start = 0.0;
      goal_node_ = nodes_pool_[0];
      goal_node_->x = g;
      goal_node_->cost_from_start = DBL_MAX; // important
      valid_tree_node_nums_ = 2;             // put start and goal in tree

      ROS_INFO("[rrt]: RRT starts planning a path");
      return rrt(s, g);
    }

    vector<Eigen::Vector3d> getPath()
    {
      return final_path_;
    }

    vector<vector<Eigen::Vector3d>> getAllPaths()
    {
      return path_list_;
    }

    vector<pair<double, double>> getSolutions()
    {
      return solution_cost_time_pair_list_;
    }

    tgk_planner::BiasSampler sampler_;

  private:
    // nodehandle params
    ros::NodeHandle nh_;

    double steer_length_;
    double search_radius_;
    double search_time_;
    int max_tree_node_nums_;
    int valid_tree_node_nums_;
    double first_path_use_time_;
    double final_path_use_time_;

    std::vector<TreeNode *> nodes_pool_;
    TreeNode *start_node_;
    TreeNode *goal_node_;
    vector<Eigen::Vector3d> final_path_;
    vector<vector<Eigen::Vector3d>> path_list_;
    vector<pair<double, double>> solution_cost_time_pair_list_;

    // environment
    tgk_planner::PosChecker::Ptr pos_checker_ptr_;

    void reset()
    {
      final_path_.clear();
      path_list_.clear();
      solution_cost_time_pair_list_.clear();
      for (int i = 0; i < valid_tree_node_nums_; i++)
      {
        nodes_pool_[i]->parent = nullptr;
        nodes_pool_[i]->children.clear();
      }
      valid_tree_node_nums_ = 0;
    }

    double calDist(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
      return (p1 - p2).norm();
    }

    Eigen::Vector3d steer(const Eigen::Vector3d &nearest_node_p, const Eigen::Vector3d &rand_node_p, double len)
    {
      Eigen::Vector3d diff_vec = rand_node_p - nearest_node_p;
      double dist = diff_vec.norm();
      if (diff_vec.norm() <= len)
        return rand_node_p;
      else
        return nearest_node_p + diff_vec * len / dist;
    }

    RRTNode3DPtr addTreeNode(RRTNode3DPtr &parent, const Eigen::Vector3d &state,
                             const double &cost_from_start, const double &cost_from_parent)
    {
      RRTNode3DPtr new_node_ptr = nodes_pool_[valid_tree_node_nums_];
      valid_tree_node_nums_++;
      new_node_ptr->parent = parent;
      parent->children.push_back(new_node_ptr);
      new_node_ptr->x = state;
      new_node_ptr->cost_from_start = cost_from_start;
      new_node_ptr->cost_from_parent = cost_from_parent;
      return new_node_ptr;
    }

    void changeNodeParent(RRTNode3DPtr &node, RRTNode3DPtr &parent, const double &cost_from_parent)
    {
      if (node->parent)
        node->parent->children.remove(node); //DON'T FORGET THIS, remove it form its parent's children list
      node->parent = parent;
      node->cost_from_parent = cost_from_parent;
      node->cost_from_start = parent->cost_from_start + cost_from_parent;
      parent->children.push_back(node);

      // for all its descedants, change the cost_from_start and tau_from_start;
      RRTNode3DPtr descendant(node);
      std::queue<RRTNode3DPtr> Q;
      Q.push(descendant);
      while (!Q.empty())
      {
        descendant = Q.front();
        Q.pop();
        for (const auto &leafptr : descendant->children)
        {
          leafptr->cost_from_start = leafptr->cost_from_parent + descendant->cost_from_start;
          Q.push(leafptr);
        }
      }
    }

    void fillPath(const RRTNode3DPtr &n, vector<Eigen::Vector3d> &path)
    {
      path.clear();
      RRTNode3DPtr node_ptr = n;
      while (node_ptr->parent)
      {
        path.push_back(node_ptr->x);
        node_ptr = node_ptr->parent;
      }
      path.push_back(start_node_->x);
      std::reverse(std::begin(path), std::end(path));
    }

    bool rrt(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
    {
      ros::Time rrt_start_time = ros::Time::now();
      bool goal_found = false;

      /* kd tree init */
      kdtree *kd_tree = kd_create(3);
      //Add start and goal nodes to kd tree
      kd_insert3(kd_tree, start_node_->x[0], start_node_->x[1], start_node_->x[2], start_node_);

      /* main loop */
      int idx = 0;
      for (idx = 0; (ros::Time::now() - rrt_start_time).toSec() < search_time_ && valid_tree_node_nums_ < max_tree_node_nums_; ++idx)
      {
        /* biased random sampling */
        Eigen::Vector3d x_rand;
        sampler_.samplingOnce(x_rand);
        if (!pos_checker_ptr_->validatePosSurround(x_rand))
        {
          continue;
        }

        struct kdres *p_nearest = kd_nearest3(kd_tree, x_rand[0], x_rand[1], x_rand[2]);
        if (p_nearest == nullptr)
        {
          ROS_ERROR("nearest query error");
          continue;
        }
        RRTNode3DPtr nearest_node = (RRTNode3DPtr)kd_res_item_data(p_nearest);
        kd_res_free(p_nearest);

        Eigen::Vector3d x_new = steer(nearest_node->x, x_rand, steer_length_);
        if (!pos_checker_ptr_->checkRayValid(nearest_node->x, x_new))
        {
          continue;
        }

        /* 1. find parent */
        /* kd_tree bounds search for parent */
        vector<RRTNode3DPtr> nearing_nodes;
        struct kdres *nbr_set;
        nbr_set = kd_nearest_range3(kd_tree, x_new[0], x_new[1], x_new[2], search_radius_);
        if (nbr_set == nullptr)
        {
          ROS_ERROR("bkwd kd range query error");
          break;
        }
        while (!kd_res_end(nbr_set))
        {
          RRTNode3DPtr curr_node = (RRTNode3DPtr)kd_res_item_data(nbr_set);
          nearing_nodes.push_back(curr_node);
          // store range query result so that we dont need to query again for rewire;
          kd_res_next(nbr_set); //go to next in kd tree range query result
        }
        kd_res_free(nbr_set); //reset kd tree range query

        /* choose parent from kd tree range query result*/
        double dist2nearest = calDist(nearest_node->x, x_new);
        double min_dist_from_start(nearest_node->cost_from_start + dist2nearest);
        double cost_from_p(dist2nearest);
        RRTNode3DPtr min_node(nearest_node); //set the nearest_node as the default parent
        for (const auto &curr_node : nearing_nodes)
        {
          if (curr_node == nearest_node) // the nearest_node already calculated and checked collision free
          {
            continue;
          }
          bool connected = pos_checker_ptr_->checkRayValid(curr_node->x, x_new);
          if (connected)
          {
            double curr_dist = calDist(curr_node->x, x_new);
            double curr_dist_from_start = curr_node->cost_from_start + curr_dist;
            if (min_dist_from_start > curr_dist_from_start)
            {
              cost_from_p = curr_dist;
              min_dist_from_start = curr_node->cost_from_start + cost_from_p;
              min_node = curr_node;
            }
          }
        }

        /* parent found within radius, then add a node to rrt and kd_tree */
        //sample rejection
        double dist_to_goal = calDist(x_new, goal_node_->x);
        if (min_dist_from_start + dist_to_goal >= goal_node_->cost_from_start)
        {
          // ROS_WARN("parent found but sample rejected");
          continue;
        }
        /* 1.1 add the randomly sampled node to rrt_tree */
        RRTNode3DPtr new_node(nullptr);
        new_node = addTreeNode(min_node, x_new, min_dist_from_start, cost_from_p);

        /* 1.2 add the randomly sampled node to kd_tree */
        kd_insert3(kd_tree, x_new[0], x_new[1], x_new[2], new_node);
        // end of find parent

        /* 2. try to connect to goal if possible */
        if (dist_to_goal <= search_radius_)
        {
          bool is_connected2goal = pos_checker_ptr_->checkRayValid(x_new, goal_node_->x);
          if (is_connected2goal && goal_node_->cost_from_start > dist_to_goal + new_node->cost_from_start) //a better path found
          {
            if (!goal_found)
            {
              first_path_use_time_ = (ros::Time::now() - rrt_start_time).toSec();
            }
            goal_found = true;
            changeNodeParent(goal_node_, new_node, dist_to_goal);
            vector<Eigen::Vector3d> curr_best_path;
            fillPath(goal_node_, curr_best_path);
            path_list_.emplace_back(curr_best_path);
            solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start, (ros::Time::now() - rrt_start_time).toSec());
          }
        }

        /* 3.rewire */
        for (auto &curr_node : nearing_nodes)
        {
          double dist_to_potential_child = calDist(new_node->x, curr_node->x);
          bool not_consistent = new_node->cost_from_start + dist_to_potential_child < curr_node->cost_from_start ? 1 : 0;
          if (not_consistent)
          {
            bool connected = pos_checker_ptr_->checkRayValid(new_node->x, curr_node->x);
            // If we can get to a node via the sampled_node faster than via it's existing parent then change the parent
            if (connected)
            {
              double best_cost_before_rewire = goal_node_->cost_from_start;
              RRTNode3DPtr old_parent(curr_node->parent);
              changeNodeParent(curr_node, new_node, dist_to_potential_child);
              if (best_cost_before_rewire > goal_node_->cost_from_start)
              {
                vector<Eigen::Vector3d> curr_best_path;
                fillPath(goal_node_, curr_best_path);
                path_list_.emplace_back(curr_best_path);
                solution_cost_time_pair_list_.emplace_back(goal_node_->cost_from_start, (ros::Time::now() - rrt_start_time).toSec());
              }
            }
          }
          /* go to the next entry */
        }
        /* end of rewire */
      }
      /* end of sample once */

      if (goal_found)
      {
        final_path_use_time_ = (ros::Time::now() - rrt_start_time).toSec();
        fillPath(goal_node_, final_path_);
        ROS_ERROR_STREAM("[rrt]: first_path_use_time: " << first_path_use_time_);
      }
      else if (valid_tree_node_nums_ == max_tree_node_nums_)
      {
        ROS_ERROR_STREAM("[rrt]: NOT CONNECTED TO GOAL after " << max_tree_node_nums_ << " nodes added to rrt-tree");
      }
      else
      {
        ROS_ERROR_STREAM("[rrt]: NOT CONNECTED TO GOAL after " << (ros::Time::now() - rrt_start_time).toSec() << " seconds");
      }
      return goal_found;
    }
  };

} // namespace path_plan
#endif