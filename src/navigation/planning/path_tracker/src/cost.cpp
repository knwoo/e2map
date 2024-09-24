#include "cost.h"


void Cost::Reset(){
    map_arrived = false;
    const_indices.clear();
}

double Cost::GetShiftCost(const Action& u, const Action& v, const std::vector<double>& cov){
    double cost = 1. / cov.at(0) * u.vx * v.vx +  1. / cov.at(1) * u.wz * v.wz;
    return cost;
}

double Cost::GetStateCost(const State& pred_state, const State& ref_state){
    auto res_state = (pred_state - ref_state).toVector();
    double cost = 0;
    for (int i = 0; i < res_state.size(); i++){
        cost += _Q.at(i) * std::pow(res_state.at(i), 2);
    }
    return cost;
}

double Cost::GetActionCost(const Action& action){
    auto vec = action.toVector();
    double cost = 0;
    for (int i = 0; i < vec.size(); i++){
        cost += _R.at(i) * std::pow(vec.at(i), 2);
    }
    return cost;
}

double Cost::GetVelocityCost(const State& pred_state, double target_vel){
    double cost = _vel_cost_weight * std::abs(pred_state.v - target_vel);
    return cost;
}

double Cost::GetSubgoalCost(const State& pred_state, const PathStamp& subgoal){
    double cost = _subgoal_cost_weight * (std::pow(subgoal.x - pred_state.x, 2) + std::pow(subgoal.y - pred_state.y, 2));
    return cost;
}

double Cost::GetHeadingCost(const geometry_msgs::Point& pred_state, const geometry_msgs::Point& subgoal, double robot_yaw){
    double path_yaw = std::atan2(subgoal.y - pred_state.y, subgoal.x - pred_state.x);
    double cost = _heading_cost_weight * abs(path_yaw - robot_yaw);
    return cost;
}

double Cost::GetTerminalCost(const State& pred_state, const State& ref_state){
    auto res_state = (pred_state - ref_state).toVector();
    double cost = 0;
    for (int i = 0; i < res_state.size(); i++){
        cost += _Qf.at(i) * std::pow(res_state.at(i), 2);
    }
    return cost;
}

double Cost::GetMapCost(const geometry_msgs::Point& pt){
    if (map_arrived == false) return 0;
    if (Cost::_CheckOutsideBound(pt)) return 1.0;
    double cost = Cost::_GetValue(pt, _cost_map);
    return cost;
}

bool Cost::GetMapConst(const geometry_msgs::Point& pt){
    if (map_arrived == false) return 0;
    if (Cost::_CheckOutsideBound(pt)) return 1.0;
    double constraint = Cost::_GetValue(pt, _cost_map);
    return constraint > _cost_thres; 
}

void Cost::SetConfig(const ros::NodeHandle& nh){
    nh.getParam(ros::this_node::getName() + "/Q", _Q);
    nh.getParam(ros::this_node::getName() + "/Qf", _Qf);
    nh.getParam(ros::this_node::getName() + "/R", _R);
    nh.getParam(ros::this_node::getName() + "/subgoal_cost_weight", _subgoal_cost_weight);    
    nh.getParam(ros::this_node::getName() + "/velocity_cost_weight", _vel_cost_weight);
    nh.getParam(ros::this_node::getName() + "/heading_cost_weight", _heading_cost_weight);
    nh.getParam(ros::this_node::getName() + "/map_cost_weight", _map_cost_weight);
    nh.getParam(ros::this_node::getName() + "/map_const", _map_const);
    nh.getParam(ros::this_node::getName() + "/cost_thres", _cost_thres);

    nh.getParam("/origin_x", _origin_x);
    nh.getParam("/origin_y", _origin_y);
    nh.getParam("/map_x_size", _map_x_size);
    nh.getParam("/map_y_size", _map_y_size);
    _bounds = {_origin_x, _origin_x + _map_x_size, _origin_y, _origin_y + _map_y_size};
}

std::vector<double> Cost::GetBound(){
    return _bounds;
}

void Cost::PrintConfig(){
    std::cout << "COST : " << std::endl;
    std::cout << "\tQ : " << _Q << std::endl;
    std::cout << "\tQf : " << _Qf << std::endl;
    std::cout << "\tR : " << _R << std::endl;
    std::cout << "\tsubgoal_cost_weight : " << _subgoal_cost_weight << std::endl;
    std::cout << "\tvelocity_cost_weight : " << _vel_cost_weight << std::endl;
    std::cout << "\theading_cost_weight : " << _heading_cost_weight << std::endl;
    std::cout << "\tmap_cost_weight : " << _map_cost_weight << std::endl;
    std::cout << "\tmap_const : " << _map_const << std::endl;
}

std::vector<double> Cost::GetQ(){
    return _Q;
}

std::vector<double> Cost::GetQf(){
    return _Qf;
}

std::vector<double> Cost::GetR(){
    return _R;
}

void Cost::SetQ(std::vector<double> Q){
    std::copy(Q.begin(), Q.end(), _Q.begin());
}

void Cost::SetQf(std::vector<double> Qf){
    std::copy(Qf.begin(), Qf.end(), _Qf.begin());
}

void Cost::SetR(std::vector<double> R){
    std::copy(R.begin(), R.end(), _R.begin());
}

double Cost::GetMapCostWeight(){
    return _map_cost_weight;
}

bool Cost::UseMapConst(){
    return _map_const;
}

void Cost::SetCost(const nav_msgs::OccupancyGrid& cost_map){
    _cost_map = cost_map;
}

bool Cost::_CheckOutsideBound(const geometry_msgs::Point& point){
    if (point.x < _bounds[0] || point.x > _bounds[1] || point.y < _bounds[2] || point.y > _bounds[3]) return true;
    return false;
}

std::vector<int> Cost::_PointToGrid(const geometry_msgs::Point& point, const nav_msgs::MapMetaData &info){
    int grid_x = (point.x - info.origin.position.x) / info.resolution;
    int grid_y = (point.y - info.origin.position.y) / info.resolution;
    grid_x = std::min(std::max(0, grid_x), (int) info.width);
    grid_y = std::min(std::max(0, grid_y), (int) info.height);

    std::vector<int> grid_xy{grid_x, grid_y};

    return grid_xy;
}

double Cost::_GetValue(const geometry_msgs::Point& point, const nav_msgs::OccupancyGrid& map){
    std::vector<int> grid_xy = Cost::_PointToGrid(point, map.info);
    int grid_idx = grid_xy[0] + map.info.width * grid_xy[1];
    bool is_out_idx = grid_idx > ((int)map.data.size() - 1);
    if (is_out_idx) throw CustomException::ErrorType::NotFoundMapValue;
    double value = std::max(map.data[grid_idx]/100.0, 0.0);
    return value;
}