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
#ifndef _NODE_UTILS_H_
#define _NODE_UTILS_H_

#include "poly_traj_utils/traj_utils.hpp"
#include <Eigen/Eigen>

using std::list;
using std::pair;
using std::vector;

#define CLOSED 'a'
#define OPEN 'b'
#define UNVISITED 'c'
#define START_TREE 's'
#define GOAL_TREE 'g'

typedef Eigen::Matrix<double, 9, 1> StatePVA;
typedef Eigen::Matrix<double, 6, 1> StatePV;
typedef Eigen::Matrix<double, 3, 1> Control;
typedef pair<double, double> BOUND;
typedef vector<BOUND> BOUNDS;

struct RRTNode
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  RRTNode *parent;
  StatePVA x;
  double cost_from_start;
  double tau_from_start;
  double cost_from_parent;
  double tau_from_parent;
  Piece poly_seg;
  list<RRTNode *> children;
  char tree_type;
  RRTNode() : parent(NULL), cost_from_start(DBL_MAX), tau_from_start(DBL_MAX), cost_from_parent(0.0), tau_from_parent(0.0), tree_type(START_TREE){};
  int countDescendant()
  {
    int n(0);
    for (const auto &child : children)
    {
      n += child->countDescendant();
    }
    return n;
  };
};
typedef RRTNode *RRTNodePtr;
typedef vector<RRTNodePtr, Eigen::aligned_allocator<RRTNodePtr>> RRTNodePtrVector;
typedef vector<RRTNode, Eigen::aligned_allocator<RRTNode>> RRTNodeVector;

class RRTNodeComparator
{
public:
  bool operator()(RRTNodePtr node1, RRTNodePtr node2)
  {
    return node1->cost_from_start > node2->cost_from_start;
  }
};

struct FMTNode
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  FMTNode *parent;
  StatePVA x;
  double g_score; //cost from start
  double f_score; //g_score + cost_to_go (heuristic estimation)
  Piece poly_seg;
  list<FMTNode *> children;
  char status;
  FMTNode() : parent(NULL), g_score(DBL_MAX), f_score(DBL_MAX), poly_seg(), status(UNVISITED){};
};
typedef FMTNode *FMTNodePtr;
typedef vector<FMTNodePtr, Eigen::aligned_allocator<FMTNodePtr>> FMTNodePtrVector;
typedef vector<FMTNode, Eigen::aligned_allocator<FMTNode>> FMTNodeVector;

class FMTNodeComparator
{
public:
  bool operator()(FMTNodePtr node1, FMTNodePtr node2)
  {
    return node1->f_score > node2->f_score;
  }
};

#endif