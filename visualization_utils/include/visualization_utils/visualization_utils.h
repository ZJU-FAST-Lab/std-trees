/*  
The MIT License

Copyright (c) 2020-2022 Jialin Ji
                        Hongkai Ye

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
#ifndef _VISUALIZE_UTILS_H_
#define _VISUALIZE_UTILS_H_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>

#define RED 1
#define BLUE 2
#define GRAY 3
#define YELLOW 4
#define TreeTraj 5
#define BLACK 6
#define GREEN 7
#define ORANGE 8
#define STEELBLUE 9
#define PINK 10
#define PURPLE 11
#define FORWARD_REACHABLE_POS 1
#define BACKWARD_REACHABLE_POS 2

typedef Eigen::Matrix<double, 9, 1> StatePVA;

class Color : public std_msgs::ColorRGBA
{
public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color()
  {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 0.7, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

class VisualRviz
{
public:
  VisualRviz();
  VisualRviz(const ros::NodeHandle &nh);

  void visualizeCollision(const Eigen::Vector3d &collision, ros::Time local_time);
  void visualizeKnots(const std::vector<Eigen::Vector3d> &knots, ros::Time local_time);
  void visualizeStates(const std::vector<StatePVA> &x, int trajectory_type, ros::Time local_time);
  void visualizeSampledState(const std::vector<StatePVA> &nodes, ros::Time local_time);
  void visualizeValidSampledState(const std::vector<StatePVA> &nodes, ros::Time local_time);
  void visualizeStartAndGoal(Eigen::Vector3d start, Eigen::Vector3d goal, ros::Time local_time);
  void visualizeTopo(const std::vector<Eigen::Vector3d> &p_head, const std::vector<Eigen::Vector3d> &tracks, ros::Time local_time);
  void visualizeOrphans(const std::vector<StatePVA> &ophs, ros::Time local_time);
  void visualizeReachPos(int type, const Eigen::Vector3d &center, const double &diam, ros::Time local_time);
  void visualizeReplanDire(const Eigen::Vector3d &pos, const Eigen::Vector3d &dire, ros::Time local_time);
  void visualizePRM(const std::vector<std::vector<Eigen::Vector3d>> &paths, Color color, ros::Time local_time);
  void visualizeLearning(const std::vector<Eigen::Vector3d> &topo_samples, const std::vector<Eigen::Vector3d> &poly_samples_, ros::Time local_time);
  void visualizeMultiTraj(const std::vector<std::vector<StatePVA>> &x, ros::Time local_time);
  void visualizeBalls(const std::vector<Eigen::Vector3d> &centers, const std::vector<double> &radii, ros::Time local_time);
  void visualizeText(const std::vector<std::string> &texts, const std::vector<Eigen::Vector3d> &positions, ros::Time local_time);
  void visualizePoints(const std::vector<StatePVA> &x, ros::Time local_time);
  void visualizeTrajList(const std::vector<std::vector<StatePVA>> &x, ros::Time local_time);
  void visualizeCurrExpectedState(const std::vector<StatePVA> &x, ros::Time local_time);

  typedef std::shared_ptr<VisualRviz> Ptr;

private:
  ros::NodeHandle nh_;
  ros::Publisher rand_sample_pos_point_pub_;
  ros::Publisher rand_sample_vel_vec_pub_;
  ros::Publisher rand_sample_acc_vec_pub_;
  ros::Publisher tree_traj_pos_point_pub_;
  ros::Publisher tree_traj_vel_vec_pub_;
  ros::Publisher tree_traj_acc_vec_pub_;
  ros::Publisher final_traj_pos_point_pub_;
  ros::Publisher final_traj_vel_vec_pub_;
  ros::Publisher final_traj_acc_vec_pub_;
  ros::Publisher first_traj_pos_point_pub_;
  ros::Publisher first_traj_vel_vec_pub_;
  ros::Publisher first_traj_acc_vec_pub_;
  ros::Publisher best_traj_pos_point_pub_;
  ros::Publisher best_traj_vel_vec_pub_;
  ros::Publisher best_traj_acc_vec_pub_;
  ros::Publisher tracked_traj_pos_point_pub_;
  ros::Publisher optimized_traj_pos_point_pub_;
  ros::Publisher optimized_traj_vel_vec_pub_;
  ros::Publisher optimized_traj_acc_vec_pub_;
  ros::Publisher fmt_traj_pos_point_pub_;
  ros::Publisher fmt_traj_vel_vec_pub_;
  ros::Publisher fmt_traj_acc_vec_pub_;
  ros::Publisher fmt_wo_traj_pos_point_pub_;
  ros::Publisher fmt_wo_traj_vel_vec_pub_;
  ros::Publisher fmt_wo_traj_acc_vec_pub_;
  ros::Publisher start_and_goal_pub_;
  ros::Publisher topo_pub_;
  ros::Publisher orphans_pos_pub_;
  ros::Publisher orphans_vel_vec_pub_;
  ros::Publisher fwd_reachable_pos_pub_;
  ros::Publisher bwd_reachable_pos_pub_;
  ros::Publisher knots_pub_;
  ros::Publisher collision_pub_;
  ros::Publisher replan_direction_pub_;
  ros::Publisher traj_list_pub_;

  ros::Publisher prm_pub_, prm_vertex_pub_, multi_traj_pos_point_pub_;
  ros::Publisher topo_pt_pub_;
  ros::Publisher poly_pt_pub_;
  ros::Publisher lines_pub_;
  ros::Publisher balls_pub_;
  ros::Publisher texts_pub_;
  ros::Publisher points_pub_;

  ros::Publisher curr_exp_pos_pub_;
  ros::Publisher curr_exp_vel_pub_;
  ros::Publisher curr_exp_acc_pub_;
};

#endif
