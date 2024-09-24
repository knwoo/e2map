#include "planner_dstar.h"

Planner::Planner() {
    _nh.getParam("/subgoal_topic", _subgoal_topic);
    _nh.getParam("/subgoal_text_topic", _subgoal_text_topic);
    _nh.getParam("/localization_topic", _localization_topic);
    _nh.getParam("/cost_map_topic", _map_topic); // should be modified to use cost_map_topic after cost_map publisher is implemented.
    _nh.getParam("/local_path_topic", _local_path_topic);
    _nh.getParam("/clicked_goal_topic", _clicked_goal_topic);
    _nh.getParam("/odom_frame_id", _frame_id);
    _nh.getParam("/subgoal_arrived_topic", _subgoal_arrived_topic);
    _nh.getParam("/reset_topic", _reset_topic);
    _nh.getParam("/collision_detection_topic", _collision_topic);
    _nh.getParam("/goal_arrived_thresh", _goal_arrived_thresh);
    
    _nh.getParam(ros::this_node::getName() + "/view_range", view_range);
    _nh.getParam(ros::this_node::getName() + "/cost_co", _cost_co);
    _nh.getParam(ros::this_node::getName() + "/enable_clicked_set", _enable_clicked_set);

    _subgoal_arrived_pub = _nh.advertise<std_msgs::Bool>(_subgoal_arrived_topic, 100);
    _path_pub = _nh.advertise<nav_msgs::Path>(_local_path_topic, 10);
    _initial_path_pub = _nh.advertise<nav_msgs::Path>("/initial_path_topic", 10);
    _vis_goal_pub = _nh.advertise<visualization_msgs::Marker>(ros::this_node::getName() + "/goal_position", 10);
    _map_sub = _nh.subscribe(_map_topic, 10, &Planner::_mapCallback, this);
    _loc_sub = _nh.subscribe(_localization_topic, 10, &Planner::_localizationCallback, this);
    _goal_sub = _nh.subscribe(_subgoal_topic, 10, &Planner::_subgoalCallback, this);
    _goal_text_sub = _nh.subscribe(_subgoal_text_topic, 10, &Planner::_subgoaltextCallback, this);
    _reset_sub = _nh.subscribe(_reset_topic, 10, &Planner::_resetCallback, this);
    _collision_sub = _nh.subscribe(_collision_topic, 10, &Planner::_collisionCallback, this);

    if (_enable_clicked_set) {
        _clicked_goal_sub = _nh.subscribe(_clicked_goal_topic, 10, &Planner::_clickedGoalCallback, this);
    }

    _init = false;
    _map_param_set = false;
    _start_set = false;
    _goal_set = false;
    _loc_msg_arrived = false;
    _map_arrived = false;
    _subgoal_changed = false;
    _moved = false;
    _subgoal_arrived.data = false;
    _clicked_subgoal_changed = false;
    _reset = false;
    _collision_occured = false;
}

void Planner::_initPlanner() {
    _dstar = new DStar(_map, _start, _goal, _cost_co);
    _scan_map = new ScanMap(_map, view_range, _cost_co);
    _init = true;
}

void Planner::_resetCallback(const std_msgs::Bool::ConstPtr &reset_msg) {
    _reset = reset_msg->data;
}    

void Planner::_collisionCallback(const std_msgs::Bool::ConstPtr &collision_msg) {
    _collision_occured = collision_msg->data;
}    

void Planner::_localizationCallback(const nav_msgs::Odometry::ConstPtr &loc_msg) {    
    if (!_start_set && _map_param_set) {
        _start = _pointToGrid(loc_msg->pose.pose.position.x, loc_msg->pose.pose.position.y);
        _start_set = true;
        _loc_msg_arrived = true;
    }
    if (_map_param_set && _robot_pos != _pointToGrid(loc_msg->pose.pose.position.x, loc_msg->pose.pose.position.y)) {
        _robot_pos = _pointToGrid(loc_msg->pose.pose.position.x, loc_msg->pose.pose.position.y);
        _loc_msg_arrived = true;
    }
}

OccupancyGridMap Planner::_transformMap(const nav_msgs::OccupancyGrid::ConstPtr &map) {
    OccupancyGridMap gridData(map);
    for (int y = 0; y < map->info.height; y++) {
        for (int x = 0; x < map->info.width; x++) {
            int index = y * map->info.width + x;
            if (static_cast<int>(map->data[index] == 0)) {gridData.occupancy_map[x][y] = 1;}
            else {gridData.occupancy_map[x][y] = static_cast<int>(map->data[index]);}
        }
    }
    return gridData;
}

void Planner::_mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map) {
    if (!_map_param_set) {
        _resolution = map->info.resolution;
        _origin_x = map->info.origin.position.x;
        _origin_y = map->info.origin.position.y;
        _map_param_set = true;
    }
    if (!_init && _goal_set && _start_set) {
        _map = _transformMap(map);
        _map_arrived = true;
        _initPlanner();
        _scan_map->setGroundTruthMap(_map);
    } else if (_init && _goal_set && _start_set) {
        _map = _transformMap(map);
        _dstar->sensed_map.setMap(_map);
        _scan_map->setGroundTruthMap(_map);
    }
}

bool Planner::_goalChanged(const geometry_msgs::Point::ConstPtr &subgoal_msg) {
    std::pair<int,int> grid_xy = _pointToGrid(subgoal_msg->x, subgoal_msg->y);
    if (_goal.first != grid_xy.first || _goal.second != grid_xy.second) return true;
    return false;
}

bool Planner::_goalChanged(const geometry_msgs::PointStamped::ConstPtr &subgoal_msg) {
    std::pair<int,int> grid_xy = _pointToGrid(subgoal_msg->point.x, subgoal_msg->point.y);
    if (_goal.first != grid_xy.first || _goal.second != grid_xy.second) return true;
    return false;
}

bool Planner::_goalChanged(const geometry_msgs::PoseStamped::ConstPtr &subgoal_msg) {
    std::pair<int,int> grid_xy = _pointToGrid(subgoal_msg->pose.position.x, subgoal_msg->pose.position.y);
    if (_goal.first != grid_xy.first || _goal.second != grid_xy.second) return true;
    return false;
}

bool Planner::_checkOutsideBound(const geometry_msgs::Point::ConstPtr& point) {
    std::pair<int,int> grid_xy = _pointToGrid(point->x, point->y);
    if (grid_xy.first < 0 || grid_xy.first > _dstar->sensed_map.x_dim || grid_xy.second < 0 || grid_xy.second > _dstar->sensed_map.y_dim) return true;
    return false;
}

void Planner::_arriveGoal() {
    if (euclideanDistance(_goal, _robot_pos) < _goal_arrived_thresh / _resolution) {
        _subgoal_arrived.data = true;
    } else {
        _subgoal_arrived.data = false;
    }
    _subgoal_arrived_pub.publish(_subgoal_arrived);
}

bool Planner::_checkOutsideBound(const geometry_msgs::PointStamped::ConstPtr& point) {
    std::pair<int,int> grid_xy = _pointToGrid(point->point.x, point->point.y);
    if (grid_xy.first < 0 || grid_xy.first > _dstar->sensed_map.x_dim || grid_xy.second < 0 || grid_xy.second > _dstar->sensed_map.y_dim) return true;
    return false;
}

bool Planner::_checkOutsideBound(const geometry_msgs::PoseStamped::ConstPtr &pose) {
    std::pair<int,int> grid_xy = _pointToGrid(pose->pose.position.x, pose->pose.position.y);
    if (grid_xy.first < 0 || grid_xy.first > _dstar->sensed_map.x_dim || grid_xy.second < 0 || grid_xy.second > _dstar->sensed_map.y_dim) return true;
    return false;
}

void Planner::_visualizeGoal() {
    visualization_msgs::Marker goal;
    visualization_msgs::Marker goal_text;
    goal.type = goal.SPHERE;
    goal.action = goal.ADD;
    goal.id = 0;
    goal.header.frame_id = _frame_id;
    goal.scale.x = 0.2;
    goal.scale.y = 0.2;
    goal.scale.z = 0.2;
    std::vector<double> goal_point = _gridToPoint(_goal);
    goal.pose.position.x = goal_point[0];
    goal.pose.position.y = goal_point[1];
    goal.pose.position.z = 0.1;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;
    goal.color.r = 1.0;
    goal.color.g = 0.0;
    goal.color.b = 0.0;
    goal.color.a = 1.0;

    goal_text.type = goal.TEXT_VIEW_FACING;
    goal_text.action = goal.ADD;
    goal_text.id = 1;
    goal_text.header.frame_id = _frame_id;
    goal_text.text = _goal_text;
    goal_text.scale.x = 0.5;
    goal_text.scale.y = 0.5;
    goal_text.scale.z = 0.5;
    goal_text.pose.position.x = goal_point[0];
    goal_text.pose.position.y = goal_point[1] + 0.3;
    goal_text.pose.position.z = 1;
    goal_text.pose.orientation.x = 0.0;
    goal_text.pose.orientation.y = 0.0;
    goal_text.pose.orientation.z = 0.0;
    goal_text.pose.orientation.w = 1.0;
    goal_text.color.r = 1.0;
    goal_text.color.g = 0.0;
    goal_text.color.b = 0.0;
    goal_text.color.a = 1.0;

    _vis_goal_pub.publish(goal);
    _vis_goal_pub.publish(goal_text);
}

void Planner::_subgoalCallback(const geometry_msgs::Point::ConstPtr &subgoal_msg) {
    if (_clicked_subgoal_changed) return;
    if (!_goal_set && _map_param_set) {
        _goal = _pointToGrid(subgoal_msg->x, subgoal_msg->y);
        _goal_set = true;
        _reset = false;
    } else if (_map_arrived) {
        if (_goalChanged(subgoal_msg)) {
            if (_checkOutsideBound(subgoal_msg)) {
                ROS_INFO("Changed goal is out of bound!");
                return;
            }
            _final_path.clear();
            _goal = _pointToGrid(subgoal_msg->x, subgoal_msg->y);
            _subgoal_changed = true;
        }
    }
}

void Planner::_subgoaltextCallback(const std_msgs::String::ConstPtr &subgoal_text_msg) {
    _goal_text = subgoal_text_msg->data;
}

void Planner::_clickedGoalCallback(const geometry_msgs::PointStamped::ConstPtr &subgoal_msg) {
    _clicked_subgoal_changed = false;
    if (!_goal_set && _map_param_set) {
        _goal = _pointToGrid(subgoal_msg->point.x, subgoal_msg->point.y);
        _goal_set = true;
    } else if (_map_arrived) {
        if (_goalChanged(subgoal_msg)) {
            if (_checkOutsideBound(subgoal_msg)) {
                ROS_INFO("Changed goal is out of bound!");
                return;
            }
            _final_path.clear();
            _goal = _pointToGrid(subgoal_msg->point.x, subgoal_msg->point.y);
            _subgoal_changed = true;
        }
    }
}

std::vector<double> Planner::_gridToPoint(const std::pair<int,int>& point) {
    double grid_x = point.first * _resolution + _origin_x;
    double grid_y = point.second * _resolution + _origin_y;
    std::vector<double> grid_xy{grid_x, grid_y};
    return grid_xy;
}

std::pair<int,int> Planner::_pointToGrid(const double x, const double y){
    int grid_x = (x - _origin_x) / _resolution;
    int grid_y = (y - _origin_y) / _resolution;
    std::pair<int,int> grid_xy{grid_x, grid_y};
    return grid_xy;
}

nav_msgs::Path Planner::_generatePathMsg() {
    nav_msgs::Path path_msg;
    std::vector<double> grid_xy;
    std::vector<double> prev_grid_xy;
    path_msg.header.frame_id = _frame_id;
    int prev_x, prev_y;
    
    for (int idx=1; idx<_final_path.size(); idx++) {
        geometry_msgs::PoseStamped pose_stamped;
        
        grid_xy = _gridToPoint(_final_path[idx]);
        
        pose_stamped.pose.position.x = grid_xy[0];
        pose_stamped.pose.position.y = grid_xy[1];
        pose_stamped.pose.position.z = 0;
        
        double yaw = 0.0;
        if (idx < _final_path.size() - 1) {
            std::vector<double> next_grid_xy = _gridToPoint(_final_path[idx+1]);
            double dx = next_grid_xy[0] - grid_xy[0];
            double dy = next_grid_xy[1] - grid_xy[1];
            yaw = atan2(dy, dx);
        } else if (idx > 0) {
            std::vector<double> prev_grid_xy = _gridToPoint(_final_path[idx-1]);
            double dx = grid_xy[0] - prev_grid_xy[0];
            double dy = grid_xy[1] - prev_grid_xy[1];
            yaw = atan2(dy, dx);
        }
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();
        if (idx != 0) {
            if (prev_x == _final_path[idx].first && prev_y == _final_path[idx].second) continue;
        }
        prev_x = _final_path[idx].first;
        prev_y = _final_path[idx].second;
        
        path_msg.poses.push_back(pose_stamped);
    }
    return path_msg;
}

void Planner::publishPath() {
    _planPath();

    nav_msgs::Path path_msg = _generatePathMsg();
    _path_pub.publish(path_msg);
    if (_init) {
        _visualizeGoal();
    }
}

void Planner::_planPath() {
    if (!_init) return;

    if (_subgoal_changed) {
        delete _dstar, _scan_map;
        _init = false;
        _start_set = false;
        _loc_msg_arrived = false;
        _map_arrived = false;
        _subgoal_changed = false;
        _moved = false;
        _map_param_set = false;
        return;
    }

    if (_reset || _collision_occured) {
        delete _dstar, _scan_map;
        _init = false;
        _start_set = false;
        _loc_msg_arrived = false;
        _map_arrived = false;
        _moved = false;
        _map_param_set = false;
        _final_path.clear();
        _goal_set = false;
        _reset = false;
        _collision_occured = false;
        return;
    }

    try {
        if (_loc_msg_arrived && _goal_set && _start_set && _init && !_moved) {
            _final_path = _dstar->plan(_robot_pos);
            _arriveGoal();
            _loc_msg_arrived = false;
            _moved = true;

            nav_msgs::Path init_path_msg = _generatePathMsg();
            _initial_path_pub.publish(init_path_msg);
        }
        else if (_loc_msg_arrived && _goal_set && _start_set && _init && _moved) {
            Vertices update_cost_and_scan_map = _scan_map->scanLocalMap(_robot_pos);

            _dstar->new_edges_and_old_costs = update_cost_and_scan_map;
            _dstar->sensed_map = _scan_map->getScanMap();

            _final_path = _dstar->plan(_robot_pos);
            _arriveGoal();

            _loc_msg_arrived = false;
        }
    } catch (const std::exception& e) {
        ROS_INFO_STREAM("Exception in Planner::_planPath: " << e.what());
    }
}

int main(int argc, char *argv[]) {
    try {
        ros::init(argc, argv, "DstarPathPlanner");
        Planner planner;
        
        ROS_INFO("LocalPathPlanner initialized.");
        ros::Rate loop_rate(10);

        while(ros::ok()) {
            ros::spinOnce();

            planner.publishPath();
            loop_rate.sleep();
        }
    } catch (const std::exception& e) {
        ROS_DEBUG_STREAM("Exception in main: " << e.what());
        return 1;
    }
    return 0;
}