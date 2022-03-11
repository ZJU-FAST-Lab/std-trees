#ifndef _GLOBAL_TRAJS_H_
#define _GLOBAL_TRAJS_H_

#include "self_msgs_and_srvs/GlbObsRcv.h"
#include "occ_grid/occ_map.h"
#include "occ_grid/pos_checker.h"
#include "kino_plan/krrt.h"
#include "kino_plan/krrt_star.h"
#include "kino_plan/krrt_sharp.h"
#include "kino_plan/bi_krrt.h"
#include "kino_plan/topo_prm.h"
#include "visualization_utils/visualization_utils.h"
#include "poly_opt/traj_optimizer.h"
#include "poly_opt/nonsmooth_trunk_opt.h"
// #include "poly_opt/trunk_opt.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolyTraj.h"
#include "quadrotor_msgs/PolyTrajs.h"
#include "r3_plan/r3_planner.h"
#include "r3_plan/a_star_search.h"
#include "kino_plan/kd_astar_jerk.h"
#include "bspline_opt/bspline_optimizer.h"
#include <bspline/non_uniform_bspline.h>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>

namespace tgk_planner
{
  class GlobalTrajs
  {
  public:
    GlobalTrajs(){};
    ~GlobalTrajs(){};
    void init(const ros::NodeHandle &nh);
    bool searchForTraj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                       Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                       double search_time);
    bool testConvergence(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                         Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                         double search_time, int deform_type, bool use_regional_opt, bool use_deform);
    bool woTest(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                double search_time, int deform_type, int planner, bool use_deform);
    bool benchmarkOptimization(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                               Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                               double search_time);

  private:
    bool optimize(const Trajectory &traj);
    void pubGlobalTrajs();
    vector<Trajectory> global_trajs_list_;

    // map, checker, planner
    OccMap::Ptr env_ptr_;
    PosChecker::Ptr pos_checker_ptr_;
    unique_ptr<TopologyPRM> topo_prm_;
    KRRTSTAR::KRRTSTARPtr krrt_star_ptr_;
    KRRT::KRRTPtr krrt_ptr_;
    KRRTSHARP::KRRTSHARPPtr krrt_sharp_ptr_;
    BIKRRT::BIKRRTPtr bikrrt_ptr_;
    TrajOptimizer::Ptr optimizer_ptr_;
    VisualRviz::Ptr vis_ptr_;
    shared_ptr<R3Planner> r3_planer_ptr_;
    shared_ptr<AstarPathFinder> astar_searcher_;
    KinodynamicAstarJ::Ptr kastar_jerk_finder_;
    fast_planner::BsplineOptimizer::Ptr bspline_optimizer_;
    shared_ptr<BranchOpt> trunk_opt_ptr_;

    // ros
    ros::NodeHandle nh_;
    ros::Timer execution_timer_;
    ros::Subscriber goal_sub_;
    ros::Publisher traj_pub_, globalTrajPub_;
    ros::ServiceClient rcv_glb_obs_client_;
    void goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr &goal_msg);
    void executionCallback(const ros::TimerEvent &event);

    // params
    Eigen::Vector3d start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, end_acc_;
    bool close_goal_traj_;
    bool new_goal_, started_, use_optimization_;
    Eigen::Vector3d last_goal_pos_;
    Trajectory front_end_traj_, back_end_traj_, traj_;
    bool map_initialized;
    double replan_time_;
    bool runned;
    double max_vel_, max_acc_, ctrl_pt_dist_;
    double rho_;

    std::ofstream o_file_;
    string file_name_;
    double start_x_, start_y_, start_z_;
    bool use_bench_, use_r3_;
    bool test_rrt_w_, test_rrt_wo_, test_a_triple_, test_bikrrt_w_, test_bikrrt_wo_;
    int repeat_times_;
    int rrt_w_succ_num_, rrt_wo_suss_num_;
    int opt_succ_num_, opt_old_succ_num_, opt_liu_succ_num_, opt_bspline_succ_num_;
    int a_double_succ_num_, a_tripla_succ_num_, birrt_w_succ_num_, birrt_wo_succ_num_;
    double avg_first_sln_time_, avg_first_sln_cost_;
    bool test_variants_, test_wo_;
  };

  void GlobalTrajs::init(const ros::NodeHandle &nh)
  {
    env_ptr_.reset(new OccMap);
    env_ptr_->init(nh);

    vis_ptr_.reset(new VisualRviz(nh));

    pos_checker_ptr_.reset(new PosChecker);
    pos_checker_ptr_->init(nh);
    pos_checker_ptr_->setMap(env_ptr_);

    astar_searcher_.reset(new AstarPathFinder());
    astar_searcher_->initGridMap(pos_checker_ptr_, pos_checker_ptr_->getOccMapSize());

    topo_prm_.reset(new TopologyPRM);
    topo_prm_->setPoschecker(pos_checker_ptr_);
    topo_prm_->init(nh);

    kastar_jerk_finder_.reset(new KinodynamicAstarJ);
    kastar_jerk_finder_->setParam(nh);
    kastar_jerk_finder_->setPosChecker(pos_checker_ptr_);
    kastar_jerk_finder_->init();

    optimizer_ptr_.reset(new TrajOptimizer(nh));
    optimizer_ptr_->setPosChecker(pos_checker_ptr_);
    optimizer_ptr_->setVisualizer(vis_ptr_);
    optimizer_ptr_->setSearcher(astar_searcher_);

    bspline_optimizer_.reset(new fast_planner::BsplineOptimizer);
    bspline_optimizer_->setParam(nh);
    bspline_optimizer_->setEnvironment(pos_checker_ptr_);

    r3_planer_ptr_.reset(new R3Planner(nh, pos_checker_ptr_));

    trunk_opt_ptr_.reset(new BranchOpt(nh));
    trunk_opt_ptr_->setPosChecker(pos_checker_ptr_);
    trunk_opt_ptr_->setVisualizer(vis_ptr_);

    krrt_star_ptr_.reset(new KRRTSTAR(nh));
    krrt_star_ptr_->init(nh);
    krrt_star_ptr_->setPosChecker(pos_checker_ptr_);
    krrt_star_ptr_->setVisualizer(vis_ptr_);
    krrt_star_ptr_->setRegionalOptimizer(optimizer_ptr_);
    krrt_star_ptr_->setSearcher(astar_searcher_);
    krrt_star_ptr_->setTrunkOptimizer(trunk_opt_ptr_);

    krrt_ptr_.reset(new KRRT(nh));
    krrt_ptr_->init(nh);
    krrt_ptr_->setPosChecker(pos_checker_ptr_);
    krrt_ptr_->setVisualizer(vis_ptr_);
    krrt_ptr_->setRegionalOptimizer(optimizer_ptr_);
    krrt_ptr_->setSearcher(astar_searcher_);
    krrt_ptr_->setTrunkOptimizer(trunk_opt_ptr_);

    krrt_sharp_ptr_.reset(new KRRTSHARP(nh));
    krrt_sharp_ptr_->init(nh);
    krrt_sharp_ptr_->setPosChecker(pos_checker_ptr_);
    krrt_sharp_ptr_->setVisualizer(vis_ptr_);
    krrt_sharp_ptr_->setRegionalOptimizer(optimizer_ptr_);
    krrt_sharp_ptr_->setSearcher(astar_searcher_);
    krrt_sharp_ptr_->setTrunkOptimizer(trunk_opt_ptr_);

    bikrrt_ptr_.reset(new BIKRRT(nh));
    bikrrt_ptr_->init(nh);
    bikrrt_ptr_->setPosChecker(pos_checker_ptr_);
    bikrrt_ptr_->setVisualizer(vis_ptr_);
    bikrrt_ptr_->setRegionalOptimizer(optimizer_ptr_);
    bikrrt_ptr_->setSearcher(astar_searcher_);
    bikrrt_ptr_->setTrunkOptimizer(trunk_opt_ptr_);

    execution_timer_ = nh_.createTimer(ros::Duration(0.01), &GlobalTrajs::executionCallback, this); // 100Hz
    traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("/front_traj", 10);
    rcv_glb_obs_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/pub_glb_obs");
    globalTrajPub_ = nh_.advertise<quadrotor_msgs::PolyTrajs>("/global_trajs", 1);

    nh.param("global_trajs/start_x", start_x_, 0.0);
    nh.param("global_trajs/start_y", start_y_, 0.0);
    nh.param("global_trajs/start_z", start_z_, 0.0);
    nh.param("global_trajs/use_optimization", use_optimization_, false);
    nh.param("global_trajs/use_r3", use_r3_, false);
    nh.param("global_trajs/replan_time", replan_time_, 0.03);
    nh.param("global_trajs/use_bench", use_bench_, false);
    nh.param("global_trajs/test_rrt_w", test_rrt_w_, false);
    nh.param("global_trajs/test_rrt_wo", test_rrt_wo_, false);
    nh.param("global_trajs/test_a_triple", test_a_triple_, false);
    nh.param("global_trajs/test_bikrrt_w", test_bikrrt_w_, false);
    nh.param("global_trajs/test_bikrrt_wo", test_bikrrt_wo_, false);
    nh.param("global_trajs/repeat_times", repeat_times_, 0);
    nh.param("global_trajs/rho", rho_, 0.01);
    nh.param("global_trajs/test_variants", test_variants_, false);
    nh.param("global_trajs/test_wo", test_wo_, false);
    //bspline
    nh.param("global_trajs/ctrl_pt_dist", ctrl_pt_dist_, 0.0);
    nh.param("global_trajs/max_vel", max_vel_, 0.0);
    nh.param("global_trajs/max_acc", max_acc_, 0.0);
    nh.param<std::string>("global_trajs/file_name", file_name_, "file");

    ROS_WARN_STREAM("[global_trajs] param: use_optimization: " << use_optimization_);
    ROS_WARN_STREAM("[global_trajs] param: replan_time: " << replan_time_);
    ROS_WARN_STREAM("[global_trajs] param: use_bench: " << use_bench_);
    ROS_WARN_STREAM("[global_trajs] param: repeat_times: " << repeat_times_);

    new_goal_ = false;
    started_ = false;
    last_goal_pos_ << start_x_, start_y_, start_z_;
    map_initialized = false;
    runned = false;

    rrt_w_succ_num_ = 0;
    rrt_wo_suss_num_ = 0;
    opt_succ_num_ = 0;
    opt_old_succ_num_ = 0;
    a_double_succ_num_ = 0;
    a_tripla_succ_num_ = 0;
    birrt_w_succ_num_ = 0;
    birrt_wo_succ_num_ = 0;
    avg_first_sln_time_ = 0;
    avg_first_sln_cost_ = 0;
  }

  void GlobalTrajs::executionCallback(const ros::TimerEvent &event)
  {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      if (!env_ptr_->mapValid())
      {
        ROS_INFO("no map.");
        self_msgs_and_srvs::GlbObsRcv srv;
        if (!rcv_glb_obs_client_.call(srv))
          ROS_WARN("Failed to call service /pub_glb_obs");
      }
      else if (!runned && use_bench_)
      {
        map_initialized = true;
        runned = true;

        // Vector3d start_pos(12.0, -12.0, 0.0), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
        // Vector3d end_pos(-12.0, 12.0, 0.0), end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
        Vector3d start_pos(-45.0, -45.0, 2.0), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
        Vector3d end_pos(45.0, 45.0, 2.0), end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
        int n(1);
        int deform_type(0);
        bool use_regional_opt(false), use_deform(false);

        deform_type = ONE_NODE;
        use_regional_opt = false;
        use_deform = true;
        n = 1;
        avg_first_sln_time_ = 0;
        avg_first_sln_cost_ = 0;
        // while (n <= repeat_times_)
        // {
        //   ROS_ERROR_STREAM("start bench No." << n << ", deform_type: " << deform_type << ", RO: " << use_regional_opt << ", deform: " << use_deform);
        //   if (testConvergence(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, use_regional_opt, use_deform))
        //     n++;
        // }
        // Trajectory tmp_traj;
        // krrt_sharp_ptr_->getTraj(tmp_traj);
        // vector<StatePVA> vis;
        // tmp_traj.sampleWholeTrajectory(&vis);
        // vis_ptr_->visualizeStates(vis, STEELBLUE, pos_checker_ptr_->getLocalTime());

        Trajectory a_star_jerk_traj;
        int res = 0;
        while (n <= repeat_times_)
        {
          kastar_jerk_finder_->reset();
          if (KinodynamicAstarJ::REACH_END == kastar_jerk_finder_->search(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, 2.0, true, 0, -1.0))
          {
            n++;
            global_trajs_list_.push_back(a_star_jerk_traj);
          }
        }
        kastar_jerk_finder_->getTraj(a_star_jerk_traj);
        double plan_time = kastar_jerk_finder_->getPlanTime();
        vector<StatePVA> vis;
        a_star_jerk_traj.sampleWholeTrajectory(&vis);
        vis_ptr_->visualizeStates(vis, STEELBLUE, pos_checker_ptr_->getLocalTime());
        pubGlobalTrajs();
      }
      fsm_num = 0;
    }
  }

  bool GlobalTrajs::testConvergence(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                                    Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                                    double search_time, int deform_type, bool use_regional_opt, bool use_deform)
  {
    bool tried_once(false);
    int result(false);
    double traj_len(0.0), traj_duration(0.0), traj_acc_itg(0.0), traj_jerk_itg(0.0);
    int traj_seg_nums(0);

    vis_ptr_->visualizeStartAndGoal(start_pos, end_pos, pos_checker_ptr_->getLocalTime());

    /* r3planner   */
    if (use_r3_)
    {
      double len_cost(0.0);
      vector<Vector3d> route;
      vector<vector<Vector3d>> routes;
      ros::Time t1 = ros::Time::now();
      double r3_use_time(0.0);
      double radius(16); //radius square
      if (r3_planer_ptr_->planOnce(start_pos, end_pos, route, len_cost, r3_use_time, radius))
      {
        tried_once = true;
        ROS_ERROR_STREAM("r3 plan solved in: " << r3_use_time * 1000 << " ms, route len: " << len_cost);
        // vector<vector<Vector3d>> select_paths;
        // size_t v_n = r3_planer_ptr_->getGraph(select_paths);
        // vis_ptr_->visualizePRM(select_paths, Color::Teal(), env_ptr_->getLocalTime());
        // getchar();
        routes.push_back(route);
        // vis_ptr_->visualizePRM(routes, Color::Red(), env_ptr_->getLocalTime());

        // vector<double> radii;
        // for (const auto & p: route)
        // {
        //   Vector3d obs;
        //   double radius = sqrt(pos_checker_ptr_->nearestObs(p, obs));
        //   radii.push_back(radius);
        // }
        // vis_ptr_->visualizeBalls(route, radii, env_ptr_->getLocalTime());
        krrt_star_ptr_->sampler_.topoSetup(routes);
        bikrrt_ptr_->sampler_.topoSetup(routes);
        krrt_ptr_->sampler_.topoSetup(routes);
        krrt_sharp_ptr_->sampler_.topoSetup(routes);
        // o_file_ << r3_use_time << "," << ","; // r3 use time (ms)
      }
      else
      {
        ROS_WARN("r3 plan fail");
        return false;
        // o_file_ << "," << ","; // r3 use time (ms)
      }
    }
    else
    {
      tried_once = true;
    }

    bool res(false);
    krrt_sharp_ptr_->reset();
    res = krrt_sharp_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, deform_type, use_regional_opt, use_deform);
    if (res == KRRTSTAR::SUCCESS)
    {
      rrt_w_succ_num_++;
      vector<double> solution_cost_list;
      vector<double> solution_time_list;
      krrt_sharp_ptr_->getConvergenceInfo(global_trajs_list_, solution_cost_list, solution_time_list);
      avg_first_sln_time_ += solution_time_list[0] * 1e3;
      avg_first_sln_cost_ += solution_cost_list[0];
      vector<vector<StatePVA>> trajs_states;
      for (size_t i = 0; i < global_trajs_list_.size(); ++i)
      {
        vector<StatePVA> single_traj_states;
        global_trajs_list_[i].sampleWholeTrajectory(&single_traj_states);
        trajs_states.push_back(single_traj_states);
      }
      pubGlobalTrajs();
      vis_ptr_->visualizeTrajList(trajs_states, pos_checker_ptr_->getLocalTime());
      return true;
    }
    else
      return false;
  }

  bool GlobalTrajs::optimize(const Trajectory &traj)
  {
    if (!optimizer_ptr_->initialize(traj, TrajOptimizer::SMOOTH_HOMO_OBS))
      return false;
    bool res = optimizer_ptr_->solve_S_H_O();
    return res;
  }

  void GlobalTrajs::pubGlobalTrajs()
  {
    if (global_trajs_list_.size() <= 0)
      return;

    quadrotor_msgs::PolyTrajs trajs;
    trajs.poly_trajs.resize(global_trajs_list_.size());
    for (size_t i = 0; i < global_trajs_list_.size(); i++)
    {
      quadrotor_msgs::PolyTraj traj_msg;
      traj_msg.hover = false;
      traj_msg.order = 5; // currently only 5-th order polynomials are supported
      vector<double> durs = global_trajs_list_[i].getDurations();
      int piece_num = global_trajs_list_[i].getPieceNum();
      traj_msg.duration.resize(piece_num);
      traj_msg.coef_x.resize(6 * piece_num);
      traj_msg.coef_y.resize(6 * piece_num);
      traj_msg.coef_z.resize(6 * piece_num);
      for (int j = 0; j < piece_num; ++j)
      {
        traj_msg.duration[j] = durs[j];
        CoefficientMat cMat = global_trajs_list_[i][j].getCoeffMat();
        int i6 = j * 6;
        for (int k = 0; k < 6; k++)
        {
          traj_msg.coef_x[i6 + k] = cMat(0, k);
          traj_msg.coef_y[i6 + k] = cMat(1, k);
          traj_msg.coef_z[i6 + k] = cMat(2, k);
        }
      }
      traj_msg.delay_time = 0.0;
      trajs.poly_trajs[i] = traj_msg;
    }
    globalTrajPub_.publish(trajs);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_node");
  ros::NodeHandle nh("~");

  tgk_planner::GlobalTrajs planners_tester;
  planners_tester.init(nh);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

#endif //_FSM_H_
