#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include <chrono>
#include <vector>
#include <random>
#include <numeric>
#include <iostream>
#include <algorithm>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <dynamic_reconfigure/server.h>
#include <path_tracker/mppi_paramConfig.h>

#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "misc.h"
#include "cost.h"
#include "action.h"
#include "state.h"
#include "dynamics.h"
#include "transform.h"
#include "dummy_path.h"
#include "regression.h"


class PathTracker{
    public:
        PathTracker(std::string pkg_loc);
        ~PathTracker();

        void MainLoop();
        void PrintConfig();
        void ResetPathTracker();

        void DynamicReconfigCallback(const path_tracker::mppi_paramConfig& config, uint32_t level);
        void SetParams(const path_tracker::mppi_paramConfig& config);

        static void SigintHandler(int sig);

        void PubTrackingInfo(double cte);
        void PubZeroAction();

        double GetCTE();

    private:
        ros::NodeHandle _nh;
        std::string _local_path_topic, _localization_topic, _cmd_ctrl_topic, _clicked_goal_topic, _cost_map_topic, _reset_topic;

        ros::Subscriber _local_path_sub;
        ros::Subscriber _localization_sub;
        ros::Subscriber _clicked_goal_sub;
        ros::Subscriber _cost_map_sub;
        ros::Subscriber _reset_sub;

        ros::Publisher _hist_pub;
        ros::Publisher _poly_pub;
        ros::Publisher _twist_pub;
        ros::Publisher _segment_pub;
        ros::Publisher _reference_pub;
        ros::Publisher _candidate_pub;
        ros::Publisher _dummy_path_pub;
        ros::Publisher _tracking_info_pub;
        ros::Publisher _lp_subgoal_pub;
        ros::Publisher _tracker_subgoal_pub;

        dynamic_reconfigure::Server<path_tracker::mppi_paramConfig> _server;
        dynamic_reconfigure::Server<path_tracker::mppi_paramConfig>::CallbackType _reconfig_fn;

        bool _verbose;
        bool _linear_path;
        bool _dynamic_config;

        std::string _custom_path_name;

        std::string _frame_id;

        bool _subgoal_changed;
        bool _is_forward;

        bool _vis_segment;
        bool _vis_reference;
        bool _vis_candidate;
        bool _vis_hist;
        bool _vis_poly;
        bool _vis_subgoal;
        bool _debug_cost;
        int _num_vis;

        bool _update_subgoal;
        bool _use_state;
        bool _use_terminal;
        bool _use_subgoal;
        bool _use_velocity;
        bool _use_heading;

        double _average;
        double _vel_coef;
        double _vel_thres_coef;

        int _K;
        int _Window;
        int _Order;
        int _Knot;
        double _Lamb;
        std::string _Regressor_Type;

        double _target_speed;
        int _n_ind_search;
        double _DL;

        double _Alp;
        double _Gam;
        double _Lam;
        std::vector<double> _Cov;

        int _target_ind;
        int _prev_ind;

        double _tracking_thres;
        int _goal_thres;
        double _subgoal_thres;

        Dynamics _dynamics;
        Cost _cost;

        std::vector<PathStamp> _path;
        Transform _trans;
        State _state;
        State _avg_state;
        double _avg_cte = 0;
        std::vector<nav_msgs::Odometry> _hist;
        std::vector<PathStamp> _path_hist;

        std::default_random_engine _gen;

        std::vector<Action> _us;
        Action _prev_u;

        DummyPathGenerator* _path_gen;

        Regressor* _path_reg;

        PathStamp _subgoal;
        geometry_msgs::Point _global_subgoal;

        void _LocalPathCallback(const nav_msgs::Path::ConstPtr& msg);
        void _LocalizationCallback(const nav_msgs::Odometry::ConstPtr& msg);
        void _ClickedGoalCallback(const geometry_msgs::PointStamped::ConstPtr& subgoal_msg);
        void _EmergencyCallback(const std_msgs::Bool::ConstPtr& msg);
        void _CostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& cost_map);
        void _ResetCallback(const std_msgs::Bool::ConstPtr& msg);
        void _PubAction(const Action& solved_action);

        void _VisPath(const std::vector<State>& path_segment);
        void _VisPath(const std::vector<PathStamp>& path_segment);
        void _VisPath(const std::vector<geometry_msgs::Point>& path_segment);
        void _VisPoly(const std::vector<geometry_msgs::Point>& path_segment);
        void _VisSegment(const std::vector<PathStamp>& path_segment);
        void _VisReference(const std::vector<geometry_msgs::Point>& ref_xs);
        void _VisCandidate(const std::vector<geometry_msgs::Point>& pred_xs, const std::vector<double>& costs, const bool constraint);
        void _VisLpSubgoal(const geometry_msgs::Point& lp_subgoal, bool marker);
        void _VisTrackerSubgoal(const geometry_msgs::Point& tracker_subgoal, bool marker);
        void _VisHist();

        std_msgs::Header _GetHeader();

        void _GeneratePath();

        void _SetConfig();

        void _UpdateState();

        std::vector<PathStamp> _GetPathSegmentBasedOnTgtVel(const std::vector<PathStamp>& path, int target_ind);
        std::vector<PathStamp> _GetLocalPathSegment(const State& state);
        
        std::vector<PathStamp> _ChangeGlobal2LocalPath(const std::vector<PathStamp>& path, const State& state);
        std::vector<PathStamp> _ChangeGlobal2Local(const std::vector<PathStamp>& path);
        std::vector<geometry_msgs::Point> _ChangeGlobal2Local(const std::vector<State>& states);

        std::vector<PathStamp> _ChangeLocal2GlobalPath(const std::vector<PathStamp>& path);
        std::vector<geometry_msgs::Point> _ChangeLocal2Global(const std::vector<PathStamp>& path);
        std::vector<geometry_msgs::Point> _ChangeLocal2Global(const std::vector<State>& states);
        geometry_msgs::Point _ChangeLocal2Global(const PathStamp& point);
        PathStamp _ChangeGlobal2Local(const geometry_msgs::Point& point);

        int _GetNearestIndex(const State& state, const std::vector<PathStamp>& path, const int prev_ind);

        std::tuple<std::vector<int>, std::vector<double> > _SortWithDistances(const std::vector<geometry_msgs::Point>& pts, const State& state);

        std::tuple<std::vector<int>, std::vector<double> > _SortWithDistances(const std::vector<PathStamp>& path, const State& state);

        std::vector<std::vector<Action> > _GetNoise();

        std::vector<double> _GetCost(const std::vector<PathStamp>& ref, const std::vector<std::vector<Action> >& noise);

        void _ClipAction(Action& action);

        State _GetRefState(const std::vector<PathStamp>& ref, const State& cur_state, int t);
        std::vector<State> _GetRefStates(const std::vector<PathStamp>& ref, const State& cur_state);

        std::vector<double> _GetWeight(const std::vector<double>& costs);

        std::vector<Action> _GetWeightedNoise(const std::vector<double>& weights, const std::vector<std::vector<Action> >& noise);

        bool _IsGoal();

        void _UpdateSolution(const std::vector<Action>& weighted_noise);
        void _UpdateSubgoal(const std::vector<PathStamp>& ref);
};

#endif