#include "path_tracker.h"


PathTracker::PathTracker(std::string pkg_loc){ 
    PathTracker::_SetConfig();

    _path_gen = new DummyPathGenerator(pkg_loc, _DL);
    if (_Regressor_Type == "Poly"){
        _path_reg = new Regressor(_Order, _Window, _DL, _verbose);
    } else if (_Regressor_Type == "Spline"){
        _path_reg = new SplineRegressor(_Knot, _Order, _Window, _DL, _verbose);
    } else if (_Regressor_Type == "SmoothSpline"){
        _path_reg = new SmoothingSplineRegressor(_Lamb,_Knot, _Order, _Window, _DL, _verbose);
    } else if (_Regressor_Type == "PenalizedBSpline"){
        _path_reg = new PenalizedBSplineRegressor(_Lamb,_Knot, _Order, _Window, _DL, _verbose);
    } else {
        _path_reg = NULL;
        throw CustomException::ErrorType::NotFoundRegressorType;
    }

    _local_path_sub = _nh.subscribe(_local_path_topic, 1, &PathTracker::_LocalPathCallback, this);
    _localization_sub = _nh.subscribe(_localization_topic, 1, &PathTracker::_LocalizationCallback, this);
    _clicked_goal_sub = _nh.subscribe(_clicked_goal_topic, 10, &PathTracker::_ClickedGoalCallback, this);
    _cost_map_sub = _nh.subscribe(_cost_map_topic, 10, &PathTracker::_CostMapCallback, this);
    _reset_sub = _nh.subscribe(_reset_topic, 10, &PathTracker::_ResetCallback, this);

    _hist_pub = _nh.advertise<nav_msgs::Path>(ros::this_node::getName() + "/history", 1);
    _poly_pub = _nh.advertise<nav_msgs::Path>(ros::this_node::getName() + "/polyline", 1);
    _twist_pub = _nh.advertise<geometry_msgs::Twist>(_cmd_ctrl_topic, 1);
    _segment_pub = _nh.advertise<nav_msgs::Path>(ros::this_node::getName() + "/segment", 1);
    _reference_pub = _nh.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName() + "/reference", 10);
    _candidate_pub = _nh.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName() + "/candidate", 10);
    _dummy_path_pub = _nh.advertise<nav_msgs::Path>(ros::this_node::getName() + "/dummy_path", 1);
    _tracking_info_pub = _nh.advertise<geometry_msgs::Point>(ros::this_node::getName() + "/tracking_info", 1);
    _lp_subgoal_pub = _nh.advertise<visualization_msgs::Marker>(ros::this_node::getName() + "/lp_subgoal", 1);
    _tracker_subgoal_pub = _nh.advertise<visualization_msgs::Marker>(ros::this_node::getName() + "/tracker_subgoal", 1);

    PathTracker::ResetPathTracker();

    PathTracker::PrintConfig();

    if (_dynamic_config){
        _reconfig_fn = boost::bind(&PathTracker::DynamicReconfigCallback, this, _1, _2);
        _server.setCallback(_reconfig_fn);
    }
}

PathTracker::~PathTracker(){
    PathTracker::ResetPathTracker();
    delete _path_gen;
    delete _path_reg;
    ROS_INFO("path_tracker is successfully deleted");
}

std::vector<PathStamp> PathTracker::_GetPathSegmentBasedOnTgtVel(const std::vector<PathStamp>& path, int target_ind) {
    std::vector<PathStamp> path_seg;
    double cum_tgt_vel_dist = _target_speed * _dynamics.GetDT();
    int idx = target_ind;
    PathStamp new_stamp = path[idx];
    path_seg.push_back(new_stamp);
    double cum_dist = sqrt(pow(path[idx+1].x - path[idx].x, 2.0) + pow(path[idx+1].y - path[idx].y, 2.0));
    while(idx < path.size()) {
        if (idx < path.size() - 1) {
            if (cum_tgt_vel_dist < cum_dist) {
                double dist = sqrt(pow(path[idx+1].x - path[idx].x, 2.0) + pow(path[idx+1].y - path[idx].y, 2.0));
                double residual = dist - cum_dist + cum_tgt_vel_dist;
                new_stamp.x = path[idx].x + residual/dist * (path[idx+1].x - path[idx].x);
                new_stamp.y = path[idx].y + residual/dist * (path[idx+1].y - path[idx].y);
                new_stamp.yaw = path[idx].yaw;
                new_stamp.gear = path[idx].gear;
                path_seg.push_back(new_stamp);
                cum_tgt_vel_dist += _target_speed * _dynamics.GetDT();
            } else {
                idx += 1;
                cum_dist += sqrt(pow(path[idx+1].x - path[idx].x, 2.0) + pow(path[idx+1].y - path[idx].y, 2.0));
            }
        } else {
            path_seg.push_back(path[idx]);
        }
        if (path_seg.size() == _dynamics.GetT()) break;
    }

    return path_seg;
}

std::vector<PathStamp> PathTracker::_GetLocalPathSegment(const State& state){
    if (!_dynamic_config) _prev_ind = 0;

    _target_ind = PathTracker::_GetNearestIndex(state, _path, _prev_ind);
    
    auto path_segment = PathTracker::_GetPathSegmentBasedOnTgtVel(_path, _target_ind);
    auto local_path_segment = PathTracker::_ChangeGlobal2Local(path_segment);

    if (_vis_segment){
        PathTracker::_VisSegment(path_segment);
    }
    _prev_ind = _target_ind;
    return local_path_segment;
}

std::vector<PathStamp> PathTracker::_ChangeGlobal2LocalPath(const std::vector<PathStamp>& path, const State& state){
    std::vector<PathStamp> local_path;
    EulerAngle e;
    e.roll = 0;
    e.pitch = 0;
    e.yaw = -state.yaw;
    auto rot = _trans.GetRotationFromEuler(e);
    std::transform(path.begin(), path.end(), std::back_inserter(local_path), [&](PathStamp stamp) -> PathStamp {
        PathStamp local_stamp;
        geometry_msgs::Point pt = _trans.ChangePointCoordinate(stamp.x - state.x, stamp.y - state.y, stamp.z - state.z, rot);
        local_stamp.x = pt.x;
        local_stamp.y = pt.y;
        local_stamp.z = pt.z;
        local_stamp.yaw = Pi2Pi(stamp.yaw - state.yaw);
        local_stamp.gear = stamp.gear;
        return local_stamp;
    });
    return local_path;
}

std::vector<PathStamp> PathTracker::_ChangeGlobal2Local(const std::vector<PathStamp>& path){
    std::vector<PathStamp> local_path;
    EulerAngle e;
    e.roll = 0;
    e.pitch = 0;
    e.yaw = -_state.yaw;
    auto rot = _trans.GetRotationFromEuler(e);
    std::transform(path.begin(), path.end(), std::back_inserter(local_path), [&](PathStamp stamp) -> PathStamp {
        PathStamp local_stamp;
        geometry_msgs::Point pt = _trans.ChangePointCoordinate(stamp.x - _state.x, stamp.y - _state.y, stamp.z - _state.z, rot);
        local_stamp.x = pt.x;
        local_stamp.y = pt.y;
        local_stamp.z = pt.z;
        local_stamp.yaw = Pi2Pi(stamp.yaw - _state.yaw);
        local_stamp.gear = stamp.gear;
        return local_stamp;
    });
    return local_path;
}

std::vector<geometry_msgs::Point> PathTracker::_ChangeGlobal2Local(const std::vector<State>& states){
    std::vector<geometry_msgs::Point> local_path;
    EulerAngle e;
    e.roll = 0;
    e.pitch = 0;
    e.yaw = -_state.yaw;
    auto rot = _trans.GetRotationFromEuler(e);
    std::transform(states.begin(), states.end(), std::back_inserter(local_path), [&](State s) -> geometry_msgs::Point {
        geometry_msgs::Point pt = _trans.ChangePointCoordinate(s.x - _state.x, s.y - _state.y, s.z - _state.z, rot);
        return pt;
    });
    return local_path;
}

std::vector<PathStamp> PathTracker::_ChangeLocal2GlobalPath(const std::vector<PathStamp>& path){
    std::vector<PathStamp> global_path;
    EulerAngle e;
    e.roll = 0;
    e.pitch = 0;
    e.yaw = _state.yaw;
    auto rot = _trans.GetRotationFromEuler(e);
    std::transform(path.begin(), path.end(), std::back_inserter(global_path), [&](PathStamp stamp) -> PathStamp {
        PathStamp global_stamp;
        geometry_msgs::Point pt = _trans.ChangePointCoordinate(stamp.x, stamp.y, stamp.z, rot);
        global_stamp.x = pt.x + _state.x;
        global_stamp.y = pt.y + _state.y;
        // global_stamp.z = pt.z + _state.z;
        global_stamp.z = pt.z;
        global_stamp.yaw = Pi2Pi(stamp.yaw + _state.yaw);
        global_stamp.gear = stamp.gear;
        return global_stamp;
    });
    return global_path;
}

std::vector<geometry_msgs::Point> PathTracker::_ChangeLocal2Global(const std::vector<PathStamp>& path){
    std::vector<geometry_msgs::Point> global_path;
    EulerAngle e;
    e.roll = 0;
    e.pitch = 0;
    e.yaw = _state.yaw;
    auto rot = _trans.GetRotationFromEuler(e);
    std::transform(path.begin(), path.end(), std::back_inserter(global_path), [&](PathStamp stamp) -> geometry_msgs::Point {
        geometry_msgs::Point global_point;
        geometry_msgs::Point pt = _trans.ChangePointCoordinate(stamp.x, stamp.y, stamp.z, rot);
        global_point.x = pt.x + _state.x;
        global_point.y = pt.y + _state.y;
        // global_point.z = pt.z + _state.z;
        global_point.z = pt.z;
        return global_point;
    });
    return global_path;
}

std::vector<geometry_msgs::Point> PathTracker::_ChangeLocal2Global(const std::vector<State>& states){
    std::vector<geometry_msgs::Point> global_path;
    EulerAngle e;
    e.roll = 0;
    e.pitch = 0;
    e.yaw = _state.yaw;
    auto rot = _trans.GetRotationFromEuler(e);
    std::transform(states.begin(), states.end(), std::back_inserter(global_path), [&](State s) -> geometry_msgs::Point {
        geometry_msgs::Point global_point;
        geometry_msgs::Point pt = _trans.ChangePointCoordinate(s.x, s.y, s.z, rot);
        global_point.x = pt.x + _state.x;
        global_point.y = pt.y + _state.y;
        // global_point.z = pt.z + _state.z;
        global_point.z = pt.z;
        return global_point;
    });
    return global_path;
}

geometry_msgs::Point PathTracker::_ChangeLocal2Global(const PathStamp& point){
    EulerAngle e;
    e.roll = 0;
    e.pitch = 0;
    e.yaw = _state.yaw;
    auto rot = _trans.GetRotationFromEuler(e);

    geometry_msgs::Point pt = _trans.ChangePointCoordinate(point.x, point.y, point.z, rot);
    geometry_msgs::Point global_point;

    global_point.x = pt.x + _state.x;
    global_point.y = pt.y + _state.y;

    return global_point;
}

PathStamp PathTracker::_ChangeGlobal2Local(const geometry_msgs::Point& point){
    EulerAngle e;
    e.roll = 0;
    e.pitch = 0;
    e.yaw = -_state.yaw;
    auto rot = _trans.GetRotationFromEuler(e);

    PathStamp local_stamp;
    geometry_msgs::Point pt = _trans.ChangePointCoordinate(point.x - _state.x, point.y - _state.y, point.z - _state.z, rot);

    local_stamp.x = pt.x;
    local_stamp.y = pt.y;
    local_stamp.z = pt.z;

    return local_stamp;
}

int PathTracker::_GetNearestIndex(const State& state, const std::vector<PathStamp>& path, const int prev_ind){
    int start_ind = prev_ind;
    int end_ind = std::min((int)path.size(), prev_ind + _n_ind_search);
    double min_distance = std::numeric_limits<double>::max();
    int min_idx;
    for (int idx = start_ind; idx < end_ind; idx++) {
        double distance = state.GetDistance(path[idx]);
        if (distance < min_distance) {
            min_distance = distance;
            min_idx = idx;
        }
    }
    return min_idx;
}

std::tuple<std::vector<int>, std::vector<double> > PathTracker::_SortWithDistances(const std::vector<geometry_msgs::Point>& pts, const State& state){
    std::vector<double> distances(pts.size());
    std::transform(pts.begin(), pts.end(), distances.begin(), [&](geometry_msgs::Point pt) -> double {
        return state.GetDistance(pt);
    });

    std::vector<int> sorted_indices(distances.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::stable_sort(sorted_indices.begin(), sorted_indices.end(), [&distances](int i, int j){
        return distances.at(i) < distances.at(j);
    });
    return std::make_tuple(sorted_indices, distances);
}

std::tuple<std::vector<int>, std::vector<double> > PathTracker::_SortWithDistances(const std::vector<PathStamp>& path, const State& state){
    std::vector<double> distances(path.size());
    std::transform(path.begin(), path.end(), distances.begin(), [&](PathStamp stamp) -> double {
        return state.GetDistance(stamp);
    });

    std::vector<int> sorted_indices(distances.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::stable_sort(sorted_indices.begin(), sorted_indices.end(), [&distances](int i, int j){
        return distances.at(i) < distances.at(j);
    });
    return std::make_tuple(sorted_indices, distances);
}

void PathTracker::ResetPathTracker(){
    _target_ind = 0;
    _prev_ind = 0;
    _is_forward = true;

    _cost.Reset();

    _us.clear();
    for (int t = 0; t < _dynamics.GetT(); t++){
        Action zero_action(0,0);
        _us.push_back(zero_action);
    }

    if (_path_gen->GetActive() == false){
        _path.clear();
    }

    _prev_u = 0;
    _avg_cte = 0;
    PathTracker::_PubAction(_prev_u);

    _hist.clear();
    _path_hist.clear();

    _path_reg->Reset();
}

void PathTracker::_SetConfig(){
    _nh.getParam("/map_frame_id", _frame_id);
    _nh.getParam("/localization_topic", _localization_topic);
    _nh.getParam("/local_path_topic", _local_path_topic);
    _nh.getParam("/cmd_ctrl_topic", _cmd_ctrl_topic);
    _nh.getParam("/clicked_goal_topic", _clicked_goal_topic);
    _nh.getParam("/cost_map_topic", _cost_map_topic);
    _nh.getParam("/reset_topic", _reset_topic);

    _nh.getParam(ros::this_node::getName() + "/verbose", _verbose);
    _nh.getParam(ros::this_node::getName() + "/average", _average);
    _nh.getParam(ros::this_node::getName() + "/vel_coef", _vel_coef);
    _nh.getParam(ros::this_node::getName() + "/vel_thres_coef", _vel_thres_coef);

    _nh.getParam(ros::this_node::getName() + "/vis_segment", _vis_segment);
    _nh.getParam(ros::this_node::getName() + "/vis_reference", _vis_reference);
    _nh.getParam(ros::this_node::getName() + "/vis_candidate", _vis_candidate);
    _nh.getParam(ros::this_node::getName() + "/vis_hist", _vis_hist);
    _nh.getParam(ros::this_node::getName() + "/vis_poly", _vis_poly);
    _nh.getParam(ros::this_node::getName() + "/vis_subgoal", _vis_subgoal);
    _nh.getParam(ros::this_node::getName() + "/num_vis", _num_vis);
    _nh.getParam(ros::this_node::getName() + "/debug_cost", _debug_cost);

    _nh.getParam(ros::this_node::getName() + "/update_subgoal", _update_subgoal);
    _nh.getParam(ros::this_node::getName() + "/use_state", _use_state);
    _nh.getParam(ros::this_node::getName() + "/use_terminal", _use_terminal);
    _nh.getParam(ros::this_node::getName() + "/use_subgoal", _use_subgoal);
    _nh.getParam(ros::this_node::getName() + "/use_velocity", _use_velocity);
    _nh.getParam(ros::this_node::getName() + "/use_heading", _use_heading);

    _nh.getParam(ros::this_node::getName() + "/K", _K);
    _nh.getParam(ros::this_node::getName() + "/Window", _Window);
    _nh.getParam(ros::this_node::getName() + "/Order", _Order);
    _nh.getParam(ros::this_node::getName() + "/Knot", _Knot);
    _nh.getParam(ros::this_node::getName() + "/Lamb", _Lamb);
    _nh.getParam(ros::this_node::getName() + "/Regressor_Type", _Regressor_Type);

    _nh.getParam(ros::this_node::getName() + "/target_speed", _target_speed);
    _nh.getParam(ros::this_node::getName() + "/n_ind_search", _n_ind_search);
    _nh.getParam(ros::this_node::getName() + "/DL", _DL);

    _nh.getParam(ros::this_node::getName() + "/Gam", _Gam);
    _nh.getParam(ros::this_node::getName() + "/Lam", _Lam);
    _nh.getParam(ros::this_node::getName() + "/Cov", _Cov);

    _nh.getParam(ros::this_node::getName() + "/custom_path_name", _custom_path_name);

    _nh.getParam(ros::this_node::getName() + "/tracking_thres", _tracking_thres);
    _nh.getParam(ros::this_node::getName() + "/goal_thres", _goal_thres);
    _nh.getParam(ros::this_node::getName() + "/subgoal_thres", _subgoal_thres);
    _nh.getParam(ros::this_node::getName() + "/linear_path", _linear_path);
    _nh.getParam(ros::this_node::getName() + "/dynamic_config", _dynamic_config);

    _Alp = 1 - _Gam / _Lam;

    _dynamics.SetConfig(_nh);
    _cost.SetConfig(_nh);

    _subgoal_changed = false;
}

void PathTracker::PrintConfig(){
    std::cout << "======================================================" << std::endl;
    std::cout << "Path Tracker : " << std::endl;

    std::cout << "\tframe_id : " << _frame_id << std::endl;
    std::cout << "\tverbose : " << _verbose << std::endl;
    std::cout << "\taverage : " << _average << std::endl;
    std::cout << "\tvel_coef : " << _vel_coef << std::endl;
    std::cout << "\tvel_thres_coef : " << _vel_thres_coef << std::endl;

    std::cout << "\tvis_segment : " << _vis_segment << std::endl;
    std::cout << "\tvis_reference : " << _vis_reference << std::endl;
    std::cout << "\tvis_candidate : " << _vis_candidate << std::endl;
    std::cout << "\tvis_hist : " << _vis_hist << std::endl;
    std::cout << "\tvis_poly : " << _vis_poly << std::endl;
    std::cout << "\tvis_subgoal : " << _vis_subgoal << std::endl;
    std::cout << "\tnum_vis : " << _num_vis << std::endl;
    std::cout << "\tdebug_cost : " << _debug_cost << std::endl;

    std::cout << "MPPI : " << std::endl;
    std::cout << "\tK : " << _K << std::endl;
    std::cout << "\tWindow : " << _Window << std::endl;
    std::cout << "\tRegressor_Type : " << _Regressor_Type << std::endl;

    std::cout << "\ttarget_speed : " << _target_speed << std::endl;
    std::cout << "\tn_ind_search : " << _n_ind_search << std::endl;
    std::cout << "\tDL : " << _DL << std::endl;
    std::cout << "\tAlp : " << _Alp << std::endl;
    std::cout << "\tGam : " << _Gam << std::endl;
    std::cout << "\tLam : " << _Lam << std::endl;
    std::cout << "\tCov : " << _Cov << std::endl;

    std::cout << "\tcustom_path_name : " << _custom_path_name << std::endl;

    std::cout << "\ttracking_thres : " << _tracking_thres << std::endl;
    std::cout << "\tgoal_thres : " << _goal_thres << std::endl;
    std::cout << "\tsubgoal_thres : " << _subgoal_thres << std::endl;

    std::cout << "\tlinear_path : " << _linear_path << std::endl;
    std::cout << "\tdynamic_config : " << _dynamic_config << std::endl;

    _dynamics.PrintConfig();
    _cost.PrintConfig();
    _path_reg->PrintConfig();
    std::cout << "======================================================" << std::endl;
}

void PathTracker::_GeneratePath(){
    if (_path_gen->flag == false){
        if (_verbose){
            std::cout << "Type : " << _path_gen->GetType() << std::endl;
        }
        std::vector<PathStamp> dummy_path;
        switch (_path_gen->GetType()){
            case PathType::ForwardStraight:
                dummy_path = _path_gen->GetForwardStraightPath();
                break;

            case PathType::BackwardStraight:
                dummy_path = _path_gen->GetBackwardStraightPath();
                break;

            case PathType::ForwardTurnRight:
                dummy_path = _path_gen->GetForwardTurnRightPath();
                break;

            case PathType::ForwardTurnLeft:
                dummy_path = _path_gen->GetForwardTurnLeftPath();
                break;

            case PathType::BackwardTurnRight:
                dummy_path = _path_gen->GetBackwardTurnRightPath();
                break;

            case PathType::BackwardTurnLeft:
                dummy_path = _path_gen->GetBackwardTurnLeftPath();
                break;

            case PathType::ForwardInfty:
                dummy_path = _path_gen->GetForwardInftyPath();
                break;

            case PathType::BackwardInfty:
                dummy_path = _path_gen->GetBackwardInftyPath();
                break;

            case PathType::ForwardRandom:
                dummy_path = _path_gen->GetForwardRandomPath();
                break;

            case PathType::BackwardRandom:
                dummy_path = _path_gen->GetBackwardRandomPath();
                break;

            case PathType::ForwardTriangle:
                dummy_path = _path_gen->GetForwardTrianglePath();
                break;

            case PathType::ForwardSquare:
                dummy_path = _path_gen->GetForwardSquarePath();
                break;

            case PathType::ForwardPentagon:
                dummy_path = _path_gen->GetForwardPentagonPath();
                break;

            case PathType::ForwardHourGlass:
                dummy_path = _path_gen->GetForwardHourGlassPath();
                break;

            case PathType::BackwardTriangle:
                dummy_path = _path_gen->GetBackwardTrianglePath();
                break;

            case PathType::BackwardSquare:
                dummy_path = _path_gen->GetBackwardSquarePath();
                break;

            case PathType::BackwardPentagon:
                dummy_path = _path_gen->GetBackwardPentagonPath();
                break;

            case PathType::BackwardHourGlass:
                dummy_path = _path_gen->GetBackwardHourGlassPath();
                break;

            case PathType::ReturnStraight:
                dummy_path = _path_gen->GetReturnStraightPath();
                break;

            case PathType::SaveCustomPath:
                PathTracker::_VisHist();
                break;

            case PathType::CustomPath:
                dummy_path = _path_gen->LoadCustomPath(_custom_path_name);
                break;

            default:
                std::cout << "Unknown Type : " << _path_gen->GetType() << std::endl;
                break;
        }

        if (_path_gen->GetType() != PathType::SaveCustomPath){
            _path = PathTracker::_ChangeLocal2GlobalPath(dummy_path);
            _path_gen->flag = true;
        }
    }
}

void PathTracker::_UpdateState(){
    auto pos = _trans.GetPos();
    auto orient =  _trans.GetOrient();
    auto twist = _trans.GetTwist();

    auto new_state = _trans.GetState(pos, orient, twist);
    auto distance = _state.GetDistance(new_state);

    if (distance > _tracking_thres){
        _state = new_state;
        throw CustomException::ErrorType::OutOdometry;
    } else {
        _state = new_state;
    }
}

void PathTracker::_UpdateSubgoal(const std::vector<PathStamp>& ref){
    auto lp_subgoal = ref.back();
    auto lp_global_subgoal = PathTracker::_ChangeLocal2Global(lp_subgoal);
    auto global_goal = _path.back();
    double angle = std::atan2(global_goal.y - _state.y, global_goal.x - _state.x);
    double r = std::sqrt(std::pow(lp_global_subgoal.x - _state.x, 2) + std::pow(lp_global_subgoal.y - _state.y, 2));

    geometry_msgs::Point tracker_subgoal;
    tracker_subgoal.x = _state.x + r * cos(angle);
    tracker_subgoal.y = _state.y + r * sin(angle);

    double dist = std::pow(lp_global_subgoal.x - tracker_subgoal.x, 2) + std::pow(lp_global_subgoal.y - tracker_subgoal.y, 2);
    bool subgoal_marker;

    if ((dist > _subgoal_thres) && _update_subgoal) {
        subgoal_marker = true;
        _subgoal = PathTracker::_ChangeGlobal2Local(tracker_subgoal);
        _global_subgoal = tracker_subgoal;
        std::cout << prColor::Yellow << "Subgoal Changed" << prColor::End << std::endl;
    }
    else {
        subgoal_marker = false;
        _subgoal = lp_subgoal;
        _global_subgoal = lp_global_subgoal;
    }
    // cylinder -> target
    if (_vis_subgoal) {
        PathTracker::_VisLpSubgoal(lp_global_subgoal, subgoal_marker);    // red
        PathTracker::_VisTrackerSubgoal(tracker_subgoal, subgoal_marker); // blue
    }
}

std::vector<std::vector<Action> > PathTracker::_GetNoise(){
    std::normal_distribution<double> dist(0.0, 1.0);
    std::vector<std::vector<Action> > noise;
    for (int k = 0; k < _K;  k++){
        std::vector<Action> noise_sequence;
        for (int i = 0; i < _dynamics.GetT(); i++){
            Action action;
            action.vx = std::sqrt(_Cov.at(0)) * dist(_gen);
            action.wz = std::sqrt(_Cov.at(1)) * dist(_gen);
            noise_sequence.push_back(action);
        }
        noise.push_back(noise_sequence);
    }

    return noise;
}

std::vector<double> PathTracker::_GetCost(const std::vector<PathStamp>& ref, const std::vector<std::vector<Action> >& noise){
    State local_state = _state.GetLocalState();

    std::vector<double> sample_costs;

    std::vector<geometry_msgs::Point> pred_xs;
    std::vector<geometry_msgs::Point> ref_xs;

    std::vector<double> state_costs;
    std::vector<double> action_costs;
    std::vector<double> shift_costs;
    std::vector<double> terminal_costs;
    std::vector<double> map_costs;

    std::vector<double> subgoal_costs;
    std::vector<double> velocity_costs;
    std::vector<double> heading_costs;       

    PathTracker::_UpdateSubgoal(ref);
    auto robot_orient = _trans.GetEulerFromQuaternion(_state.orient);

    if (_path_reg->CheckDOF(ref.size())){
        _path_reg->Fit(ref);
    } else{
        throw CustomException::ErrorType::NotInitalizeRegressor;
    }
    if (_path_reg->CheckSolved() == false){
        throw CustomException::ErrorType::NotSolvedRegressor;
    }

    _cost.const_indices.clear();
    for (int k = 0; k < _K; k++){
        State cur_state = local_state;
        State ref_state;

        double state_cost = 0;
        double velocity_cost = 0;
        double action_cost = 0;
        double shift_cost = 0;
        double map_cost = 0;
        bool map_const = false;

        int end_ind = std::min((int)_dynamics.GetT(), (int)ref.size());
        for (int t = 0; t < end_ind; t++){
            Action v;
            auto u = _us.at(t);
            auto e = noise.at(k).at(t);

            if (k < (1 - _Alp) * _K){
                v = u + e;
            } else {
                v = e;
            }

            PathTracker::_ClipAction(v);

            cur_state = _dynamics.UpdateState(cur_state, v);
            ref_state = PathTracker::_GetRefState(ref, cur_state, t);

            state_cost += _cost.GetStateCost(cur_state, ref_state);
            action_cost += _cost.GetActionCost(v);
            shift_cost += _Gam * _cost.GetShiftCost(u, v, _Cov);
            velocity_cost += _cost.GetVelocityCost(cur_state, _target_speed);

            auto global_ref_state = ref_state.toGlobalPoint(_state.yaw, _state.toGeoPoint());
            ref_xs.push_back(global_ref_state);
            auto global_state = cur_state.toGlobalPoint(_state.yaw, _state.toGeoPoint());
            pred_xs.push_back(global_state);

            map_cost += _cost.GetMapCost(global_state);
            map_const = map_const || _cost.GetMapConst(global_state);
        }
        if (map_const) _cost.const_indices.push_back(k);

        auto global_cur_state = cur_state.toGlobalPoint(_state.yaw, _state.toGeoPoint());
        double heading_cost = _cost.GetHeadingCost(global_cur_state, _global_subgoal, robot_orient.yaw);
        double terminal_cost = _cost.GetTerminalCost(cur_state, ref_state);
        double subgoal_cost = _cost.GetSubgoalCost(cur_state, _subgoal);
    
        map_cost = _cost.GetMapCostWeight() * map_cost;

        state_costs.push_back(state_cost);
        action_costs.push_back(action_cost);
        shift_costs.push_back(shift_cost);
        terminal_costs.push_back(terminal_cost);
        map_costs.push_back(map_cost);
        subgoal_costs.push_back(subgoal_cost);
        velocity_costs.push_back(velocity_cost);
        heading_costs.push_back(heading_cost);

        if (!_use_state) state_cost = 0;
        if (!_use_subgoal) subgoal_cost = 0;
        if (!_use_terminal) terminal_cost = 0;
        if (!_use_velocity) velocity_cost = 0;
        if (!_use_heading) heading_cost = 0;

        double total_cost = state_cost + action_cost + shift_cost + terminal_cost + map_cost + subgoal_cost + velocity_cost + heading_cost;

        sample_costs.push_back(total_cost);
    }

    if (_vis_candidate){
        PathTracker::_VisCandidate(pred_xs, subgoal_costs, _cost.UseMapConst());
    }
    if (_vis_reference){
        PathTracker::_VisReference(ref_xs);
    }
    if (_vis_poly){
        auto line = _path_reg->Getline(ref);
        auto global_line = PathTracker::_ChangeLocal2Global(line);
        PathTracker::_VisPoly(global_line);
    }
    if (_debug_cost && _verbose){
        std::cout << std::fixed << std::setprecision(2);
        std::cout << prColor::Cyan << "Cost Info : " << prColor::End << std::endl;
        std::cout << std::left;
        std::cout << prColor::Cyan << std::setw(20) << "\tState Cost" << ": (" << GetMean(state_costs) << ", " << GetStd(state_costs) << ")" << prColor::End << std::endl;
        std::cout << prColor::Cyan << std::setw(20) << "\tAction Cost" << ": (" << GetMean(action_costs) << ", " << GetStd(action_costs) << ")" << prColor::End << std::endl;
        std::cout << prColor::Cyan << std::setw(20) << "\tShift Cost" << ": (" << GetMean(shift_costs) << ", " << GetStd(shift_costs) << ")" << prColor::End << std::endl;
        std::cout << prColor::Cyan << std::setw(20) << "\tTerminal Cost" << ": (" << GetMean(terminal_costs) << ", " << GetStd(terminal_costs) << ")" << prColor::End << std::endl;
        std::cout << prColor::Cyan << std::setw(20) << "\tMap Cost" << ": (" << GetMean(map_costs) << ", " << GetStd(map_costs) << ")" << prColor::End << std::endl;
        std::cout << prColor::Cyan << std::setw(20) << "\tSubgoal Cost" << ": (" << GetMean(subgoal_costs) << ", " << GetStd(subgoal_costs) << ")" << prColor::End << std::endl;
        std::cout << prColor::Cyan << std::setw(20) << "\tVel Cost" << ": (" << GetMean(velocity_costs) << ", " << GetStd(velocity_costs) << ")" << prColor::End << std::endl;
        std::cout << prColor::Cyan << std::setw(20) << "\tHeading Cost" << ": (" << GetMean(heading_costs) << ", " << GetStd(heading_costs) << ")" << prColor::End << std::endl;

        std::cout << prColor::Cyan << std::setw(20) << "\tConst Candidate" << ": " << _cost.const_indices << prColor::End << std::endl;
    }

    return sample_costs;
}

void PathTracker::_ClipAction(Action& action){
    double maxV = _dynamics.GetMaxLinVel();
    double maxW = _dynamics.GetMaxAngVel();
    action.vx = std::max(std::min(action.vx, maxV), -maxV);
    action.wz = std::max(std::min(action.wz, maxW), -maxW);
}

State PathTracker::_GetRefState(const std::vector<PathStamp>& ref, const State& cur_state, int t){
    auto target_yaw = _path_reg->GetTangentialAngle(ref.at(t).x);

    State ref_state;
    ref_state.x = ref.at(t).x;
    ref_state.y = ref.at(t).y;
    ref_state.v = ref.at(t).gear * _target_speed;
    ref_state.yaw = target_yaw;

    return ref_state;
}

std::vector<double> PathTracker::_GetWeight(const std::vector<double>& costs){
    double rho = *std::min_element(costs.begin(), costs.end());

    double eta = 0;
    auto weight_fn = [&](double cost) -> double {
        double weight = std::exp(- 1 / _Lam * (cost - rho));
        eta += weight;
        return weight;
    };

    std::vector<double> weights;
    std::transform(costs.begin(), costs.end(), std::back_inserter(weights), weight_fn);

    auto normalize_fn = [&](double weight) -> double {
        return  weight / eta;
    };

    std::vector<double> normalized_weights;
    std::transform(weights.begin(), weights.end(), std::back_inserter(normalized_weights), normalize_fn);

    if (_cost.UseMapConst()){
        for (const auto& idx : _cost.const_indices){
            normalized_weights.at(idx) = 0;
        }
    }

    return normalized_weights;
}

std::vector<Action> PathTracker::_GetWeightedNoise(const std::vector<double>& weights, const std::vector<std::vector<Action> >& noise){
    int n_size = _dynamics.GetT();
    std::vector<Action> weighted_noise;

    for (int t = 0; t < _dynamics.GetT(); t++){
        Action weight_noise_t;
        weight_noise_t = 0;
        for (int k = 0; k < _K; k++){
            weight_noise_t += noise.at(k).at(t) * weights.at(k);
        }
        weighted_noise.push_back(weight_noise_t);
    }

    return weighted_noise;
}

void PathTracker::_UpdateSolution(const std::vector<Action>& weighted_noise){
    std::vector<Action> solution(_us.size());
    for (int t = 0; t < _dynamics.GetT(); t++){
        solution.at(t) = _us.at(t) + weighted_noise.at(t);
    }

    PathTracker::_PubAction(solution.at(0));

    for (int t = 1; t < _dynamics.GetT(); t++){
        _us.at(t-1) = solution.at(t);
    }

    _us.at(_dynamics.GetT() -1) = solution.at(_dynamics.GetT() -1);

    _prev_u = _us.at(0);
}

double PathTracker::GetCTE(){

    double eps = 1e-6;

    int next_target_ind = std::min(_target_ind + 1, (int)_path.size() - 1);

    std::vector<double> direction{_path.at(next_target_ind).x - _path.at(_target_ind).x, _path.at(next_target_ind).y - _path.at(_target_ind).y};
    double direction_norm = std::sqrt(std::pow(direction.at(0), 2) + std::pow(direction.at(1), 2));
    std::vector<double> to_robot{_state.x - _path.at(_target_ind).x, _state.y - _path.at(_target_ind).y};
    double to_robot_norm = std::sqrt(std::pow(to_robot.at(0), 2) + std::pow(to_robot.at(1), 2));
    double cte = std::abs(to_robot.at(0) * direction.at(1) - to_robot.at(1) * direction.at(0)) / (direction_norm + eps);


    int prev_target_ind = std::max(_target_ind - 1, 0);
    std::vector<double> prev_direction{_path.at(_target_ind).x - _path.at(prev_target_ind).x, _path.at(_target_ind).y - _path.at(prev_target_ind).y};
    double prev_direction_norm = std::sqrt(std::pow(prev_direction.at(0), 2) + std::pow(prev_direction.at(1), 2));
    std::vector<double> prev_to_robot{_state.x - _path.at(prev_target_ind).x, _state.y - _path.at(prev_target_ind).y};
    double prev_to_robot_norm = std::sqrt(std::pow(prev_to_robot.at(0), 2) + std::pow(prev_to_robot.at(1), 2));
    double prev_cte = std::abs(prev_to_robot.at(0) * prev_direction.at(1) - prev_to_robot.at(1) * prev_direction.at(0)) / (prev_direction_norm + eps);

    double error = std::min(cte, prev_cte);
    return error;
}

bool PathTracker::_IsGoal(){
    bool is_goal = std::abs((int)_path.size() - 1 - _target_ind) <= _goal_thres;
    return is_goal;
}

void PathTracker::MainLoop(){
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    PathTracker::_UpdateState();

    if (_path_gen->GetActive()){
        PathTracker::_GeneratePath();
        if (_path.size() > 1){
            PathTracker::_VisHist();
            PathTracker::_VisPath(_path);
        }
    }

    // minimum path size is 2 due to the path slicing
    if (_path.size() < 2){
        throw CustomException::ErrorType::NotImplementedPath;
    }

    auto is_goal = PathTracker::_IsGoal();
    if (is_goal){
        throw CustomException::ErrorType::GoalArrived;
    }

    auto cte = PathTracker::GetCTE();
    PathTracker::PubTrackingInfo(cte);
    if (cte > _tracking_thres){
        throw CustomException::ErrorType::NotFoundTargetPath;
    }

    auto path_segment = PathTracker::_GetLocalPathSegment(_state);

    auto noise = PathTracker::_GetNoise();

    auto costs = PathTracker::_GetCost(path_segment, noise);

    auto weights = PathTracker::_GetWeight(costs);

    auto weighted_noise = PathTracker::_GetWeightedNoise(weights, noise);

    PathTracker::_UpdateSolution(weighted_noise);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    auto elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    if (_verbose){
        std::cout << "Current State :" << std::endl;
        std::cout << _state << std::endl;

        std::cout << "Average State :" << std::endl;
        std::cout << _avg_state << std::endl;

        std::cout << std::setw(20) << "Current Ind" << ": " << _target_ind << " / " << _path.size() - 1 << std::endl;
        std::cout << std::setw(20) << "Elapsed Time" << ": " << elapsed_time << " [ms]" << std::endl;
        std::cout << "---" << std::endl;
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "path_tracker_node", ros::init_options::NoSigintHandler);

    std::string pkg_loc = ros::package::getPath("path_tracker");

    PathTracker controller(pkg_loc);
    signal(SIGINT, PathTracker::SigintHandler);

    ROS_INFO("%spath_tracker is successfully initilaized%s", prColor::Cyan, prColor::End);

    ros::Rate LoopRate(10);
    while(ros::ok()){
        ros::spinOnce();
        try {
            controller.MainLoop();

        } catch (std::exception& e){
            controller.~PathTracker();
            std::cout << typeid(e).name() << std::endl;
            std::cerr << e.what() << std::endl;

        } catch (CustomException::ErrorType& e){
            CustomException::report(e);
            controller.ResetPathTracker();
        }

        LoopRate.sleep();
    }
    return 0;
}