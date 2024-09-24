#pragma once

#include "dstar.h"
#include "occupancy_grid_map.h"
#include "utils.h"

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf2/LinearMath/Quaternion.h>

class Planner {
    public:
        Planner();
        void publishPath();
    private:
        void _initPlanner();
        void _localizationCallback(const nav_msgs::Odometry::ConstPtr &loc_msg);
        void _mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
        bool _goalChanged(const geometry_msgs::Point::ConstPtr &subgoal_msg);
        bool _goalChanged(const geometry_msgs::PointStamped::ConstPtr &subgoal_msg);
        bool _goalChanged(const geometry_msgs::PoseStamped::ConstPtr &subgoal_msg);
        void _subgoalCallback(const geometry_msgs::Point::ConstPtr &subgoal_msg);
        void _subgoaltextCallback(const std_msgs::String::ConstPtr &subgoal_text_msg);
        void _clickedGoalCallback(const geometry_msgs::PointStamped::ConstPtr &subgoal_msg);
        void _resetCallback(const std_msgs::Bool::ConstPtr &reset_msg);
        void _collisionCallback(const std_msgs::Bool::ConstPtr &collision_msg);
        bool _checkOutsideBound(const geometry_msgs::Point::ConstPtr& point);
        bool _checkOutsideBound(const geometry_msgs::PoseStamped::ConstPtr& pose);
        void _arriveGoal();
        bool _checkOutsideBound(const geometry_msgs::PointStamped::ConstPtr& point);
        void _visualizeGoal();
        void _planPath();

        std::vector<double> _gridToPoint(const std::pair<int,int>& point);
        std::pair<int,int> _pointToGrid(const double x, const double y);
        nav_msgs::Path _generatePathMsg();
        OccupancyGridMap _transformMap(const nav_msgs::OccupancyGrid::ConstPtr &map);
        
        std::pair<int, int> _start;
        std::pair<int, int> _goal;
        std::pair<int, int> _robot_pos;
        std::vector<std::pair<int, int>> _final_path;
        OccupancyGridMap _map;
        std::vector<std::vector<int>> empty_map;

        DStar* _dstar;
        ScanMap*  _scan_map;

        bool _init;
        bool _map_param_set;
        bool _start_set;
        bool _goal_set;
        bool _loc_msg_arrived;
        bool _map_arrived;
        bool _subgoal_changed;
        bool _moved;
        bool _clicked_subgoal_changed;
        bool _enable_clicked_set;
        bool _reset;
        bool _collision_occured;
        int _margin;
        int view_range;
        double _cost_co;
        double _resolution;
        double _origin_x;
        double _origin_y;
        double _goal_arrived_thresh;
        std_msgs::Bool _subgoal_arrived;

        ros::NodeHandle _nh;

        ros::Publisher _path_pub;
        ros::Publisher _initial_path_pub;
        ros::Publisher _subgoal_arrived_pub;
        ros::Publisher _vis_goal_pub;
        ros::Subscriber _map_sub;
        ros::Subscriber _loc_sub;
        ros::Subscriber _goal_sub;
        ros::Subscriber _goal_text_sub;
        ros::Subscriber _clicked_goal_sub;
        ros::Subscriber _reset_sub;
        ros::Subscriber _collision_sub;

        std::string _subgoal_topic;
        std::string _subgoal_text_topic;
        std::string _localization_topic;
        std::string _map_topic;
        std::string _local_path_topic;
        std::string _initial_path_topic;
        std::string _clicked_goal_topic;
        std::string _subgoal_arrived_topic;
        std::string _reset_topic;
        std::string _collision_topic;
        std::string _frame_id;
        std::string _goal_text;
};
