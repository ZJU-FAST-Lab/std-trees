#ifndef _TEST_PLANNERS_H_
#define _TEST_PLANNERS_H_

#include "self_msgs_and_srvs/GlbObsRcv.h"
#include "occ_grid/occ_map.h"
#include "occ_grid/pos_checker.h"
#include "kino_plan/krrt.h"
#include "kino_plan/krrt_star.h"
#include "kino_plan/krrt_sharp.h"
#include "kino_plan/topo_prm.h"
#include "visualization_utils/visualization_utils.h"
#include "poly_opt/traj_optimizer.h"
#include "poly_opt/nonsmooth_trunk_opt.h"
// #include "poly_opt/trunk_opt.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "r3_plan/rrt_star.h"
#include "r3_plan/r3_planner.h"
#include "r3_plan/a_star_search.h"
#include "kino_plan/kd_astar_jerk.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Empty.h>

namespace tgk_planner
{
  struct BenchmarkResult
  {
    // Evaluation settings
    int trial_number = -1;
    std::string method_name = "none";

    // Trajectory settings
    int num_segments = 0;
    double nominal_length = 0.0;

    // Evaluation results
    int optimization_success = 0;
    bool bounds_violated = false;
    double trajectory_time = 0.0;
    double trajectory_length = 0.0;
    double computation_time = 0.0; // in s
    double j_max = 0.0;
    double v_max = 0.0;
    double a_max = 0.0;
    double cost = 0.0;
    double max_dist_from_straight_line = 0.0;
    double area_traj_straight_line = 0.0;

    double acceleration_integral = 0.0;
    double jerk_integral = 0.0;

    double corridor_time = 0.0; // in s
  };

  class PlannerTester
  {
  public:
    PlannerTester(){};
    ~PlannerTester(){};
    void init(const ros::NodeHandle &nh);
    bool testConvergence(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                         Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                         double search_time, int deform_type, bool use_regional_opt, bool use_deform);
    bool woTest(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                double search_time, int deform_type, int planner, bool use_deform);
    BenchmarkResult runCorridorBasedOnce(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                                         Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                                         const vector<Eigen::Vector3d> &rrt_path);
    BenchmarkResult runRichterOnce(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                                   Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                                   const vector<Eigen::Vector3d> &rrt_path);
    void benchForCorridorBased(int n, Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                               Vector3d end_pos, Vector3d end_vel, Vector3d end_acc);

  private:
    // map, checker, planner
    OccMap::Ptr env_ptr_;
    PosChecker::Ptr pos_checker_ptr_;
    KRRTSTAR::KRRTSTARPtr krrt_star_ptr_;
    std::shared_ptr<BVPSolver::IntegratorBVP> bvpPtr_;
    KRRT::KRRTPtr krrt_ptr_;
    KRRTSHARP::KRRTSHARPPtr krrt_sharp_ptr_;
    TrajOptimizer::Ptr optimizer_ptr_;
    VisualRviz::Ptr vis_ptr_;
    shared_ptr<R3Planner> r3_planer_ptr_;
    shared_ptr<path_plan::RRTStar> rrt_star_path_planner_ptr_;
    shared_ptr<AstarPathFinder> astar_searcher_;
    KinodynamicAstarJ::Ptr kastar_jerk_finder_;
    shared_ptr<BranchOpt> trunk_opt_ptr_;

    // ros
    ros::NodeHandle nh_;
    ros::Timer execution_timer_;
    ros::Subscriber goal_sub_;
    ros::Publisher traj_pub_;
    ros::Publisher hPolyPub_;
    ros::Publisher path_marker_pub_;
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
    double max_vel_, max_acc_, max_jerk_, ctrl_pt_dist_;
    double rho_;

    const static int kN_ = 12; // has to be even !!
    int kDim_ = 3;
    bool print_debug_info_;
    double collision_check_dt_;
    int max_loop_num_;
    double bbox_width_;

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
    bool test_variants_, test_wo_, test_hierarchical_;
    double start_pos_x_, start_pos_y_, start_pos_z_, end_pos_x_, end_pos_y_, end_pos_z_;
  };

  void PlannerTester::init(const ros::NodeHandle &nh)
  {
    env_ptr_.reset(new OccMap);
    env_ptr_->init(nh);

    vis_ptr_.reset(new VisualRviz(nh));

    pos_checker_ptr_.reset(new PosChecker);
    pos_checker_ptr_->init(nh);
    pos_checker_ptr_->setMap(env_ptr_);

    astar_searcher_.reset(new AstarPathFinder());
    astar_searcher_->initGridMap(pos_checker_ptr_, pos_checker_ptr_->getOccMapSize());

    kastar_jerk_finder_.reset(new KinodynamicAstarJ);
    kastar_jerk_finder_->setParam(nh);
    kastar_jerk_finder_->setPosChecker(pos_checker_ptr_);
    kastar_jerk_finder_->init();

    optimizer_ptr_.reset(new TrajOptimizer(nh));
    optimizer_ptr_->setPosChecker(pos_checker_ptr_);
    optimizer_ptr_->setVisualizer(vis_ptr_);
    optimizer_ptr_->setSearcher(astar_searcher_);

    bvpPtr_ = std::make_shared<BVPSolver::IntegratorBVP>();

    r3_planer_ptr_.reset(new R3Planner(nh, pos_checker_ptr_));

    rrt_star_path_planner_ptr_.reset(new path_plan::RRTStar(nh, pos_checker_ptr_));

    trunk_opt_ptr_.reset(new BranchOpt(nh));
    trunk_opt_ptr_->setPosChecker(pos_checker_ptr_);
    trunk_opt_ptr_->setVisualizer(vis_ptr_);

    krrt_star_ptr_.reset(new KRRTSTAR(nh, bvpPtr_));
    krrt_star_ptr_->init(nh);
    krrt_star_ptr_->setPosChecker(pos_checker_ptr_);
    krrt_star_ptr_->setVisualizer(vis_ptr_);
    krrt_star_ptr_->setRegionalOptimizer(optimizer_ptr_);
    krrt_star_ptr_->setSearcher(astar_searcher_);
    krrt_star_ptr_->setTrunkOptimizer(trunk_opt_ptr_);

    krrt_ptr_.reset(new KRRT(nh, bvpPtr_));
    krrt_ptr_->init(nh);
    krrt_ptr_->setPosChecker(pos_checker_ptr_);
    krrt_ptr_->setVisualizer(vis_ptr_);
    krrt_ptr_->setRegionalOptimizer(optimizer_ptr_);
    krrt_ptr_->setSearcher(astar_searcher_);
    krrt_ptr_->setTrunkOptimizer(trunk_opt_ptr_);

    krrt_sharp_ptr_.reset(new KRRTSHARP(nh, bvpPtr_));
    krrt_sharp_ptr_->init(nh);
    krrt_sharp_ptr_->setPosChecker(pos_checker_ptr_);
    krrt_sharp_ptr_->setVisualizer(vis_ptr_);
    krrt_sharp_ptr_->setRegionalOptimizer(optimizer_ptr_);
    krrt_sharp_ptr_->setSearcher(astar_searcher_);
    krrt_sharp_ptr_->setTrunkOptimizer(trunk_opt_ptr_);

    execution_timer_ = nh_.createTimer(ros::Duration(0.01), &PlannerTester::executionCallback, this); // 100Hz
    goal_sub_ = nh_.subscribe("/goal", 1, &PlannerTester::goalCallback, this);
    traj_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>("/front_traj", 10);
    rcv_glb_obs_client_ = nh_.serviceClient<self_msgs_and_srvs::GlbObsRcv>("/pub_glb_obs");
    path_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path", 1, true);

    nh.param("test_plan/start_x", start_x_, 0.0);
    nh.param("test_plan/start_y", start_y_, 0.0);
    nh.param("test_plan/start_z", start_z_, 0.0);
    nh.param("test_plan/use_optimization", use_optimization_, false);
    nh.param("test_plan/use_r3", use_r3_, false);
    nh.param("test_plan/replan_time", replan_time_, 0.03);
    nh.param("test_plan/use_bench", use_bench_, false);
    nh.param("test_plan/test_rrt_w", test_rrt_w_, false);
    nh.param("test_plan/test_rrt_wo", test_rrt_wo_, false);
    nh.param("test_plan/test_a_triple", test_a_triple_, false);
    nh.param("test_plan/test_bikrrt_w", test_bikrrt_w_, false);
    nh.param("test_plan/test_bikrrt_wo", test_bikrrt_wo_, false);
    nh.param("test_plan/repeat_times", repeat_times_, 0);
    nh.param("test_plan/rho", rho_, 0.01);
    nh.param("test_plan/test_variants", test_variants_, false);
    nh.param("test_plan/test_wo", test_wo_, false);
    nh.param("test_plan/test_hierarchical", test_hierarchical_, false);

    nh.param("test_plan/start_pos_x", start_pos_x_, 0.0);
    nh.param("test_plan/start_pos_y", start_pos_y_, 0.0);
    nh.param("test_plan/start_pos_z", start_pos_z_, 0.0);
    nh.param("test_plan/end_pos_x", end_pos_x_, 0.0);
    nh.param("test_plan/end_pos_y", end_pos_y_, 0.0);
    nh.param("test_plan/end_pos_z", end_pos_z_, 0.0);

    //bspline
    nh.param("test_plan/ctrl_pt_dist", ctrl_pt_dist_, 0.0);
    nh.param("test_plan/max_vel", max_vel_, 0.0);
    nh.param("test_plan/max_acc", max_acc_, 0.0);
    nh.param("test_plan/max_jerk", max_jerk_, 0.0);
    nh.param<std::string>("test_plan/file_name", file_name_, "file");

    nh.param("test_plan/print_debug_info", print_debug_info_, false);
    nh.param("test_plan/collision_check_dt", collision_check_dt_, 0.0);
    nh.param("test_plan/max_loop_num", max_loop_num_, 0);
    nh.param("test_plan/bbox_width", bbox_width_, 0.0);

    ROS_WARN_STREAM("[test_plan] param: use_optimization: " << use_optimization_);
    ROS_WARN_STREAM("[test_plan] param: replan_time: " << replan_time_);
    ROS_WARN_STREAM("[test_plan] param: use_bench: " << use_bench_);
    ROS_WARN_STREAM("[test_plan] param: repeat_times: " << repeat_times_);

    bvpPtr_->init(TRIPLE_INTEGRATOR);
    bvpPtr_->setRho(rho_);

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

  void PlannerTester::executionCallback(const ros::TimerEvent &event)
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

        /*********  bench convergence  ************/
        if (test_variants_)
        {
          // Vector3d start_pos(12.0, -12.0, 0.0), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
          // Vector3d end_pos(-12.0, 12.0, 0.0), end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
          Vector3d start_pos(0.0, -32.0, 0.0), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
          Vector3d end_pos(0.0, 32.0, 0.0), end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
          int n(1);
          int deform_type(0);
          bool use_regional_opt(false), use_deform(false);
          o_file_.open(file_name_.append("-one-node.csv"), ios::out | ios::app);

          deform_type = ONE_NODE;
          use_regional_opt = false;
          use_deform = true;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << ", deform_type: " << deform_type << ", RO: " << use_regional_opt << ", deform: " << use_deform);
            if (testConvergence(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, use_regional_opt, use_deform))
              n++;
          }
          // avg_first_sln_time_ /= repeat_times_;
          // avg_first_sln_cost_ /= repeat_times_;
          // o_file_ << avg_first_sln_time_ << "," << avg_first_sln_cost_ << ",";
          // o_file_ << endl;
          o_file_.close();
          Trajectory tmp_traj;
          krrt_sharp_ptr_->getTraj(tmp_traj);
          vector<StatePVA> vis;
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, STEELBLUE, pos_checker_ptr_->getLocalTime());

          file_name_ = file_name_.substr(0, file_name_.length() - 13);
          o_file_.open(file_name_.append("-trunk.csv"), ios::out | ios::app);
          deform_type = TRUNK_NODES;
          use_regional_opt = false;
          use_deform = true;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << ", deform_type: " << deform_type << ", RO: " << use_regional_opt << ", deform: " << use_deform);
            if (testConvergence(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, use_regional_opt, use_deform))
              n++;
          }
          // avg_first_sln_time_ /= repeat_times_;
          // avg_first_sln_cost_ /= repeat_times_;
          // o_file_ << avg_first_sln_time_ << "," << avg_first_sln_cost_ << ",";
          o_file_.close();
          krrt_sharp_ptr_->getTraj(tmp_traj);
          vis.clear();
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, ORANGE, pos_checker_ptr_->getLocalTime());

          file_name_ = file_name_.substr(0, file_name_.length() - 10);
          o_file_.open(file_name_.append("-branch.csv"), ios::out | ios::app);
          deform_type = BRANCH_NODES;
          use_regional_opt = false;
          use_deform = true;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << ", deform_type: " << deform_type << ", RO: " << use_regional_opt << ", deform: " << use_deform);
            if (testConvergence(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, use_regional_opt, use_deform))
              n++;
          }
          // avg_first_sln_time_ /= repeat_times_;
          // avg_first_sln_cost_ /= repeat_times_;
          // o_file_ << avg_first_sln_time_ << "," << avg_first_sln_cost_ << ",";
          o_file_.close();

          file_name_ = file_name_.substr(0, file_name_.length() - 11);
          o_file_.open(file_name_.append("-small-branch.csv"), ios::out | ios::app);
          deform_type = SMALL_BRANCH_NODES;
          use_regional_opt = false;
          use_deform = true;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << ", deform_type: " << deform_type << ", RO: " << use_regional_opt << ", deform: " << use_deform);
            if (testConvergence(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, use_regional_opt, use_deform))
              n++;
          }
          // avg_first_sln_time_ /= repeat_times_;
          // avg_first_sln_cost_ /= repeat_times_;
          // o_file_ << avg_first_sln_time_ << "," << avg_first_sln_cost_ << ",";
          o_file_.close();
          krrt_sharp_ptr_->getTraj(tmp_traj);
          vis.clear();
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, RED, pos_checker_ptr_->getLocalTime());

          file_name_ = file_name_.substr(0, file_name_.length() - 17);
          o_file_.open(file_name_.append("-tree.csv"), ios::out | ios::app);
          deform_type = TREE_NODES;
          use_regional_opt = false;
          use_deform = true;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << ", deform_type: " << deform_type << ", RO: " << use_regional_opt << ", deform: " << use_deform);
            if (testConvergence(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, use_regional_opt, use_deform))
              n++;
          }
          // avg_first_sln_time_ /= repeat_times_;
          // avg_first_sln_cost_ /= repeat_times_;
          // o_file_ << avg_first_sln_time_ << "," << avg_first_sln_cost_ << ",";
          o_file_.close();
          krrt_sharp_ptr_->getTraj(tmp_traj);
          vis.clear();
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, GREEN, pos_checker_ptr_->getLocalTime());

          file_name_ = file_name_.substr(0, file_name_.length() - 9);
          o_file_.open(file_name_.append("-no-deform.csv"), ios::out | ios::app);
          deform_type = NO_DEFORM;
          use_regional_opt = false;
          use_deform = false;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << ", deform_type: " << deform_type << ", RO: " << use_regional_opt << ", deform: " << use_deform);
            if (testConvergence(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, use_regional_opt, use_deform))
              n++;
          }
          // avg_first_sln_time_ /= repeat_times_;
          // avg_first_sln_cost_ /= repeat_times_;
          // o_file_ << avg_first_sln_time_ << "," << avg_first_sln_cost_ << ",";
          o_file_.close();
          krrt_sharp_ptr_->getTraj(tmp_traj);
          vis.clear();
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, PURPLE, pos_checker_ptr_->getLocalTime());

          cout << "00000: " << endl;
          if (test_a_triple_)
          {
            file_name_ = file_name_.substr(0, file_name_.length() - 14);
            o_file_.open(file_name_.append("-astar.csv"), ios::out | ios::app);
            int res = 0;
            double lambda_heu[] = {1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0};
            Trajectory a_star_jerk_traj;
            double plan_time(0.0);
            double cost(0.0);
            double curr_traj_len(0.0), curr_traj_duration(0.0), curr_traj_acc_itg(0.0), curr_traj_jerk_itg(0.0);
            int curr_traj_seg_nums(0);
            int n(0);
            std::vector<StatePVA> vis_x;
            int discre_num = sizeof(lambda_heu) / sizeof(lambda_heu[0]);
            for (int i = 0; i < discre_num; ++i)
            {
              n = 0;
              res = 0;
              while (res != KinodynamicAstarJ::REACH_END)
              {
                n++;
                cout << "lambda_heu: " << lambda_heu[i] << ", Kinodynamic a star start " << n << endl;
                kastar_jerk_finder_->reset();
                res = kastar_jerk_finder_->search(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, lambda_heu[i], true, 0, -1.0);
                kastar_jerk_finder_->getTraj(a_star_jerk_traj);
                plan_time = kastar_jerk_finder_->getPlanTime();
              }
              vis_x.clear();
              a_star_jerk_traj.sampleWholeTrajectory(&vis_x);
              vis_ptr_->visualizeStates(vis_x, BLUE, pos_checker_ptr_->getLocalTime());
              cost = krrt_star_ptr_->evaluateTraj(a_star_jerk_traj, curr_traj_duration, curr_traj_len, curr_traj_seg_nums, curr_traj_acc_itg, curr_traj_jerk_itg);
              o_file_ << lambda_heu[i] << ","
                      << plan_time * 1e3 << ","
                      << cost << ","
                      << endl;
              cout << "lambda_heu: " << lambda_heu[i] << ", Kinodynamic a star jerk end" << endl;
            }

            o_file_.close();
          }
        }
        /*********  bench convergence  ************/

        if (test_wo_)
        {
          Vector3d start_pos(start_pos_x_, start_pos_y_, start_pos_z_), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
          Vector3d end_pos(end_pos_x_, end_pos_y_, end_pos_z_), end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
          // Vector3d start_pos(0.0, -35.0, 0.0), start_vel(0.0, 0.0, 0.0), start_acc(0.0, 0.0, 0.0);
          // Vector3d end_pos(0.0, 35.0, 0.0), end_vel(0.0, 0.0, 0.0), end_acc(0.0, 0.0, 0.0);
          int n(1);
          int deform_type(0);
          int planner_type(0);
          bool use_deform(false);
          Trajectory tmp_traj;
          vector<StatePVA> vis;
          o_file_.open(file_name_.append("-rrt-w.csv"), ios::out | ios::app);

          deform_type = SMALL_BRANCH_NODES;

          planner_type = 1;
          use_deform = true;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << "planner_type: " << planner_type << ", deform: " << use_deform);
            if (woTest(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, planner_type, use_deform))
              n++;
          }
          o_file_.close();
          krrt_ptr_->getTraj(tmp_traj);
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, BLUE, pos_checker_ptr_->getLocalTime());

          file_name_ = file_name_.substr(0, file_name_.length() - 10);
          o_file_.open(file_name_.append("-rrt-wo.csv"), ios::out | ios::app);
          planner_type = 1;
          use_deform = false;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << "planner_type: " << planner_type << ", deform: " << use_deform);
            if (woTest(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, planner_type, use_deform))
              n++;
          }
          o_file_.close();
          krrt_ptr_->getTraj(tmp_traj);
          vis.clear();
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, STEELBLUE, pos_checker_ptr_->getLocalTime());

          file_name_ = file_name_.substr(0, file_name_.length() - 11);
          o_file_.open(file_name_.append("-rrtStar-w.csv"), ios::out | ios::app);
          planner_type = 2;
          use_deform = true;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << "planner_type: " << planner_type << ", deform: " << use_deform);
            if (woTest(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, planner_type, use_deform))
              n++;
          }
          o_file_.close();
          krrt_star_ptr_->getTraj(tmp_traj);
          vis.clear();
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, GREEN, pos_checker_ptr_->getLocalTime());

          file_name_ = file_name_.substr(0, file_name_.length() - 14);
          o_file_.open(file_name_.append("-rrtStar-wo.csv"), ios::out | ios::app);
          planner_type = 2;
          use_deform = false;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << "planner_type: " << planner_type << ", deform: " << use_deform);
            if (woTest(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, planner_type, use_deform))
              n++;
          }
          o_file_.close();
          krrt_star_ptr_->getTraj(tmp_traj);
          vis.clear();
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, YELLOW, pos_checker_ptr_->getLocalTime());

          file_name_ = file_name_.substr(0, file_name_.length() - 15);
          o_file_.open(file_name_.append("-rrtSharp-w.csv"), ios::out | ios::app);
          planner_type = 3;
          use_deform = true;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << "planner_type: " << planner_type << ", deform: " << use_deform);
            if (woTest(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, planner_type, use_deform))
              n++;
          }
          o_file_.close();
          krrt_sharp_ptr_->getTraj(tmp_traj);
          vis.clear();
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, RED, pos_checker_ptr_->getLocalTime());

          file_name_ = file_name_.substr(0, file_name_.length() - 15);
          o_file_.open(file_name_.append("-rrtSharp-wo.csv"), ios::out | ios::app);
          planner_type = 3;
          use_deform = false;
          n = 1;
          avg_first_sln_time_ = 0;
          avg_first_sln_cost_ = 0;
          while (n <= repeat_times_)
          {
            ROS_ERROR_STREAM("start bench No." << n << "planner_type: " << planner_type << ", deform: " << use_deform);
            if (woTest(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, replan_time_, deform_type, planner_type, use_deform))
              n++;
          }
          o_file_.close();
          krrt_sharp_ptr_->getTraj(tmp_traj);
          vis.clear();
          tmp_traj.sampleWholeTrajectory(&vis);
          vis_ptr_->visualizeStates(vis, ORANGE, pos_checker_ptr_->getLocalTime());
        }
      }
      fsm_num = 0;
    }
  }

  void PlannerTester::goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr &goal_msg)
  {
    end_pos_ << goal_msg->position.x,
        goal_msg->position.y,
        goal_msg->position.z;
    end_vel_ << goal_msg->velocity.x,
        goal_msg->velocity.y,
        goal_msg->velocity.z;
    end_acc_ << goal_msg->acceleration.x,
        goal_msg->acceleration.y,
        goal_msg->acceleration.z;
    started_ = true;
    new_goal_ = true;
    last_goal_pos_ = end_pos_;
  }

  bool PlannerTester::testConvergence(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                                      Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                                      double search_time, int deform_type, bool use_regional_opt, bool use_deform)
  {
    bool tried_once(false);
    int result(false);
    double traj_len(0.0), traj_duration(0.0), traj_acc_itg(0.0), traj_jerk_itg(0.0);
    int traj_seg_nums(0);

    vis_ptr_->visualizeStartAndGoal(start_pos, end_pos, pos_checker_ptr_->getLocalTime());

    Eigen::VectorXd head(9), tail(9);
    head.head(3) = start_pos;
    head.segment(3, 3) = start_vel;
    head.tail(3) = start_acc;
    tail.head(3) = end_pos;
    tail.segment(3, 3) = end_vel;
    tail.tail(3) = end_acc;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> traversal_lines;
    if (bvpPtr_->solve(head, tail, ACC_KNOWN))
    {
      double bvp_total_T = bvpPtr_->getTauStar();
      CoefficientMat bvp_coeff;
      bvpPtr_->getCoeff(bvp_coeff);
      Piece bvp_piece(bvp_total_T, bvp_coeff);
      pos_checker_ptr_->checkPolySeg(bvp_piece, traversal_lines);
    }
    r3_planer_ptr_->sampler_.topoSetup(traversal_lines, start_pos, end_pos);
    krrt_ptr_->sampler_.topoSetup(traversal_lines, start_pos, end_pos);
    krrt_star_ptr_->sampler_.topoSetup(traversal_lines, start_pos, end_pos);
    krrt_sharp_ptr_->sampler_.topoSetup(traversal_lines, start_pos, end_pos);

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
        double r3_use_time = (ros::Time::now() - t1).toSec() * 1e3;
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
      vector<Trajectory> traj_list;
      vector<double> solution_cost_list;
      vector<double> solution_time_list;
      krrt_sharp_ptr_->getConvergenceInfo(traj_list, solution_cost_list, solution_time_list);
      avg_first_sln_time_ += solution_time_list[0] * 1e3;
      avg_first_sln_cost_ += solution_cost_list[0];
      vector<vector<StatePVA>> trajs_states;
      for (size_t i = 0; i < traj_list.size(); ++i)
      {
        vector<StatePVA> single_traj_states;
        traj_list[i].sampleWholeTrajectory(&single_traj_states);
        trajs_states.push_back(single_traj_states);
        double curr_traj_len(0.0), curr_traj_duration(0.0), curr_traj_acc_itg(0.0), curr_traj_jerk_itg(0.0);
        int curr_traj_seg_nums(0);
        krrt_sharp_ptr_->evaluateTraj(traj_list[i], curr_traj_duration, curr_traj_len, curr_traj_seg_nums, curr_traj_acc_itg, curr_traj_jerk_itg);
        o_file_ << i + 1 << ","
                << solution_time_list[i] * 1e3 << ","
                // << curr_traj_seg_nums << ","
                // << curr_traj_acc_itg << ","
                // << curr_traj_jerk_itg << ","
                << solution_cost_list[i] << ","
                // << curr_traj_duration << ","
                // << curr_traj_len << ","
                << endl;
      }
      vis_ptr_->visualizeTrajList(trajs_states, pos_checker_ptr_->getLocalTime());
      // o_file_ << endl;
      return true;
    }
    else
      return false;
  }

  bool PlannerTester::woTest(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                             Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                             double search_time, int deform_type, int planner_type, bool use_deform)
  {
    bool tried_once(false);
    int result(false);
    double traj_len(0.0), traj_duration(0.0), traj_acc_itg(0.0), traj_jerk_itg(0.0);
    int traj_seg_nums(0);

    vis_ptr_->visualizeStartAndGoal(start_pos, end_pos, pos_checker_ptr_->getLocalTime());

    Eigen::VectorXd head(9), tail(9);
    head.head(3) = start_pos;
    head.segment(3, 3) = start_vel;
    head.tail(3) = start_acc;
    tail.head(3) = end_pos;
    tail.segment(3, 3) = end_vel;
    tail.tail(3) = end_acc;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> traversal_lines;
    if (bvpPtr_->solve(head, tail, ACC_KNOWN))
    {
      double bvp_total_T = bvpPtr_->getTauStar();
      CoefficientMat bvp_coeff;
      bvpPtr_->getCoeff(bvp_coeff);
      Piece bvp_piece(bvp_total_T, bvp_coeff);
      pos_checker_ptr_->checkPolySeg(bvp_piece, traversal_lines);
    }
    rrt_star_path_planner_ptr_->sampler_.topoSetup(traversal_lines, start_pos, end_pos);
    krrt_ptr_->sampler_.topoSetup(traversal_lines, start_pos, end_pos);
    krrt_star_ptr_->sampler_.topoSetup(traversal_lines, start_pos, end_pos);
    krrt_sharp_ptr_->sampler_.topoSetup(traversal_lines, start_pos, end_pos);

    bool res(false);
    switch (planner_type)
    {
    case 1:
      krrt_ptr_->reset();
      res = krrt_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, deform_type, 0, use_deform);
      break;

    case 2:
      krrt_star_ptr_->reset();
      res = krrt_star_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, deform_type, 0, use_deform);
      break;

    case 3:
      krrt_sharp_ptr_->reset();
      res = krrt_sharp_ptr_->plan(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, search_time, deform_type, 0, use_deform);
      break;

    default:
      break;
    }

    if (res == KRRTSTAR::SUCCESS)
    {
      rrt_w_succ_num_++;
      vector<Trajectory> traj_list;
      vector<double> solution_cost_list;
      vector<double> solution_time_list;
      switch (planner_type)
      {
      case 1:
        krrt_ptr_->getConvergenceInfo(traj_list, solution_cost_list, solution_time_list);
        break;

      case 2:
        krrt_star_ptr_->getConvergenceInfo(traj_list, solution_cost_list, solution_time_list);
        break;

      case 3:
        krrt_sharp_ptr_->getConvergenceInfo(traj_list, solution_cost_list, solution_time_list);
        break;

      default:
        break;
      }
      avg_first_sln_time_ += solution_time_list[0] * 1e3;
      avg_first_sln_cost_ += solution_cost_list[0];
      vector<vector<StatePVA>> trajs_states;
      for (size_t i = 0; i < traj_list.size(); ++i)
      {
        vector<StatePVA> single_traj_states;
        traj_list[i].sampleWholeTrajectory(&single_traj_states);
        trajs_states.push_back(single_traj_states);
        double curr_traj_len(0.0), curr_traj_duration(0.0), curr_traj_acc_itg(0.0), curr_traj_jerk_itg(0.0);
        int curr_traj_seg_nums(0);
        krrt_star_ptr_->evaluateTraj(traj_list[i], curr_traj_duration, curr_traj_len, curr_traj_seg_nums, curr_traj_acc_itg, curr_traj_jerk_itg);
        o_file_ << i + 1 << ","
                << solution_time_list[i] * 1e3 << ","
                << solution_cost_list[i] << ","
                << curr_traj_duration << ","
                << endl;
      }
      vis_ptr_->visualizeTrajList(trajs_states, pos_checker_ptr_->getLocalTime());
      // o_file_ << endl;
      res = true;
    }
    else
      res = false;

    return res;
  }

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine_node");
  ros::NodeHandle nh("~");
  tgk_planner::PlannerTester planners_tester;
  planners_tester.init(nh);

  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::waitForShutdown();
}

#endif //_FSM_H_
