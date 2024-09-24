#include <path_tracker.h>
// #include <filesystem>


void PathTracker::_LocalPathCallback(const nav_msgs::Path::ConstPtr& msg){
    _path.clear();
    for (int i = 0 ; i < msg->poses.size(); i ++){
        PathStamp stamp;
        stamp.x = msg->poses[i].pose.position.x;
        stamp.y = msg->poses[i].pose.position.y;
        stamp.z = msg->poses[i].pose.position.z;

        auto angle = _trans.GetEulerFromQuaternion(msg->poses[i].pose.orientation);
        stamp.yaw = angle.yaw;
        stamp.gear = 1.0;
        _path.push_back(stamp);
    }
    // std::cout <<  _path << std::endl;
}

void PathTracker::_LocalizationCallback(const nav_msgs::Odometry::ConstPtr& msg){
    _trans.SetPos(msg->pose.pose.position);
    _trans.SetOrient(msg->pose.pose.orientation);
    _trans.SetTwist(msg->twist.twist);

    if(isinf(msg->twist.twist.linear.x) || isinf(msg->twist.twist.linear.y) || isinf(msg->twist.twist.linear.z)) {
        geometry_msgs::Twist dummy;
        _trans.SetTwist(dummy);
        ROS_WARN("INF WARN from current velocity!\n");
    }

    if (_path_gen->GetActive()){
        _hist.push_back(*msg);
    } else{
        if (_vis_hist && (_path.size() > 2)){
            _hist.push_back(*msg);
            _path_hist.push_back(_path.at(_target_ind));
            PathTracker::_VisPath(_path_hist);
            PathTracker::_VisHist();
        } else{
            _hist.clear();
            _path_hist.clear();
        }
    }

    // std::cout << "pos : " << std::endl;
    // std::cout << _pos << std::endl;

    // std::cout << "orient : " << std::endl;
    // std::cout << _orient << std::endl;

    // std::cout << "twist : " << std::endl;
    // std::cout << _twist << std::endl;
}

void PathTracker::_ClickedGoalCallback(const geometry_msgs::PointStamped::ConstPtr& subgoal_msg){
    _subgoal_changed = true;
    if (_subgoal_changed){
        PathTracker::ResetPathTracker();
        _subgoal_changed = false;
    }
    std::cout << prColor::Yellow << "Goal Changed" << prColor::End << std::endl;
}

void PathTracker::_CostMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& cost_map) {
    _cost.SetCost(*cost_map);
    _cost.map_arrived = true;
}

void PathTracker::_ResetCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        PathTracker::ResetPathTracker();
    }
}

void PathTracker::_PubAction(const Action& solved_action){
    geometry_msgs::Twist twistMsg;

    // twistMsg.linear.x = solved_action.vx * _dynamics.GetMaxLinVel();
    // twistMsg.angular.z = solved_action.wz * _dynamics.GetMaxAngVel();
    twistMsg.linear.x = solved_action.vx;
    twistMsg.angular.z = solved_action.wz;

    _twist_pub.publish(twistMsg);
}

void PathTracker::DynamicReconfigCallback(const path_tracker::mppi_paramConfig& config, uint32_t level){
    if (config.Dummy_Path){
        _path_gen->SetActive(config.Dummy_Activate);
        if (config.Dummy_Activate == false){
            PathTracker::ResetPathTracker();
            if (_path_gen->GetType() == PathType::SaveCustomPath){
                if (_hist.size() > 0){
                    auto path_info = _path_gen->ProcessCustomPath(_trans, _hist);
                    auto custom_path = PathTracker::_ChangeGlobal2LocalPath(std::get<0>(path_info), std::get<1>(path_info));
                    _path_gen->SaveCustomPath(custom_path, _custom_path_name);
                } else {
                    ROS_INFO("No Path to Save");
                }
            }

            _hist.clear();
        }
    }

    _path_gen->SetType(config.Dummy_Type);
    _path_gen->SetCurvature(config.Dummy_Curvature);
    _path_gen->SetLength(config.Dummy_Length);
    _path_gen->PrintConfig();

   PathTracker::SetParams(config);
   PathTracker::PrintConfig();
}

void PathTracker::SetParams(const path_tracker::mppi_paramConfig& config){
    _verbose = config.verbose;
    _path_reg->SetVerbose(config.verbose);
    _vis_hist = config.vis_hist;
    _K = config.K;
    _vel_coef = config.vel_coef;
    _vel_thres_coef = config.vel_thres_coef;
    _average = config.average;

    _dynamics.SetT(config.T);
    _n_ind_search = config.n_ind_search;
    _dynamics.SetDT(config.DT);
    _target_speed = config.target_speed;

    _Lam = config.Lam;
    _Gam = config.Gam;
    _Alp = 1 - _Gam / _Lam;

    _Cov.at(0) = config.Cov_Lin_Vel;
    _Cov.at(1) = config.Cov_Ang_Vel;

    std::vector<double> Q(4);
    Q.at(0) = config.Q_Pos;
    Q.at(1) = config.Q_Pos;
    Q.at(2) = config.Q_Vel;
    Q.at(3) = config.Q_Yaw;
    _cost.SetQ(Q);
    std::vector<double> Qf(4);
    Qf.at(0) = config.Qf_Pos;
    Qf.at(1) = config.Qf_Pos;
    Qf.at(2) = config.Qf_Vel;
    Qf.at(3) = config.Qf_Yaw;
    _cost.SetQf(Qf);
    std::vector<double> R(2);
    R.at(0) = config.R_Lin_Vel;
    R.at(1) = config.R_Ang_Vel;
    _cost.SetR(R);
}

void PathTracker::PubTrackingInfo(double cte){
    auto tracking_info_msg = geometry_msgs::Point();
    tracking_info_msg.x = cte;
    tracking_info_msg.y = _target_speed;
    tracking_info_msg.z = _state.v;
    _tracking_info_pub.publish(tracking_info_msg);
    if (_verbose){
        _avg_state = _avg_state * (_average - 1) / _average  + _state * 1 / _average;
        _avg_cte = _avg_cte * (_average - 1) / _average  + cte * 1 / _average;

        std::cout <<  prColor::Yellow << "Tracking Info : " << prColor::End << std::endl;
        std::cout <<  prColor::Yellow << std::setw(20) << "\tCur CTE" <<  ": " << cte << prColor::End << std::endl;
        std::cout <<  prColor::Yellow << std::setw(20) << "\tCur Vel" <<  ": " << _state.v << prColor::End << std::endl;
        std::cout <<  prColor::Yellow << std::setw(20) << "\tAvg CTE" <<  ": " << _avg_cte << prColor::End << std::endl;
        std::cout <<  prColor::Yellow << std::setw(20) << "\tAvg Vel" <<  ": " << _avg_state.v << prColor::End << std::endl;
    }
}

void PathTracker::SigintHandler(int sig){

    ros::NodeHandle nh;
    std::string cmd_ctrl_topic;
    nh.getParam("/cmd_ctrl_topic", cmd_ctrl_topic);
    
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>(cmd_ctrl_topic, 1);

    geometry_msgs::Twist twistMsg;

    twistMsg.linear.x = 0;
    twistMsg.angular.z = 0;

    twist_pub.publish(twistMsg);

    ROS_INFO("%sKill path_tracker_node%s, signal : %d", prColor::Red, prColor::End, sig);

    ros::shutdown();
}

void PathTracker::PubZeroAction(){
    Action zero(0,0);
    PathTracker::_PubAction(zero);
}