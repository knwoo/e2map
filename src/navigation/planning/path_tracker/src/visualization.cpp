#include <path_tracker.h>


void PathTracker::_VisHist(){
    nav_msgs::Path path_msg;
    for (const auto& stamp : _hist){
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.pose.position.x = stamp.pose.pose.position.x;
        pose_stamp.pose.position.y = stamp.pose.pose.position.y;
        pose_stamp.pose.position.z = stamp.pose.pose.position.z;
        path_msg.poses.push_back(pose_stamp);
    }
    path_msg.header = PathTracker::_GetHeader();
    _hist_pub.publish(path_msg);
}

void PathTracker::_VisPath(const std::vector<State>& path_segment){
    nav_msgs::Path path_msg;
    for (const auto& stamp : path_segment){
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.pose.position.x = stamp.x;
        pose_stamp.pose.position.y = stamp.y;
        pose_stamp.pose.position.z = stamp.z;
        path_msg.poses.push_back(pose_stamp);
    }
    path_msg.header = PathTracker::_GetHeader();
    _dummy_path_pub.publish(path_msg);
}

void PathTracker::_VisPath(const std::vector<PathStamp>& path_segment){
    nav_msgs::Path path_msg;
    for (const auto& stamp : path_segment){
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.pose.position.x = stamp.x;
        pose_stamp.pose.position.y = stamp.y;
        pose_stamp.pose.position.z = stamp.z;
        path_msg.poses.push_back(pose_stamp);
    }
    path_msg.header = PathTracker::_GetHeader();
    _dummy_path_pub.publish(path_msg);
}

void PathTracker::_VisPath(const std::vector<geometry_msgs::Point>& path_segment){
    nav_msgs::Path path_msg;
    for (const auto& stamp : path_segment){
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.pose.position.x = stamp.x;
        pose_stamp.pose.position.y = stamp.y;
        pose_stamp.pose.position.z = stamp.z;
        path_msg.poses.push_back(pose_stamp);
    }
    path_msg.header = PathTracker::_GetHeader();
    _dummy_path_pub.publish(path_msg);
}

void PathTracker::_VisPoly(const std::vector<geometry_msgs::Point>& path_segment){
    nav_msgs::Path path_msg;
    for (const auto& stamp : path_segment){
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.pose.position.x = stamp.x;
        pose_stamp.pose.position.y = stamp.y;
        pose_stamp.pose.position.z = stamp.z;
        path_msg.poses.push_back(pose_stamp);
    }
    path_msg.header = PathTracker::_GetHeader();
    _poly_pub.publish(path_msg);
}

void PathTracker::_VisSegment(const std::vector<PathStamp>& path_segment){
    nav_msgs::Path path_msg;
    for (const auto& stamp : path_segment){
        geometry_msgs::PoseStamped pose_stamp;
        pose_stamp.pose.position.x = stamp.x;
        pose_stamp.pose.position.y = stamp.y;
        pose_stamp.pose.position.z = stamp.z;
        path_msg.poses.push_back(pose_stamp);
    }
    path_msg.header = PathTracker::_GetHeader();
    _segment_pub.publish(path_msg);
}

void PathTracker::_VisReference(const std::vector<geometry_msgs::Point>& ref_xs){
    int T = (int)_dynamics.GetT();
    int num_path = std::min(_num_vis, _K);
    std::vector<std::vector<geometry_msgs::Point> > resized_xs;
    std::vector<geometry_msgs::Point> point_seq;
    for (const auto& xs: ref_xs){
        point_seq.push_back(xs);
        if (point_seq.size() == T){
            resized_xs.push_back(point_seq);
            point_seq.clear();
            if (resized_xs.size() == num_path){
                break;
            }
        }
    }

    visualization_msgs::MarkerArray ref_msg;
    auto header = PathTracker::_GetHeader();
    int id = 0;
    for (const auto& xs: resized_xs){
        visualization_msgs::Marker line_strip;
        line_strip.header = header;
        line_strip.id = id++;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.12;
        line_strip.color.r = 255 / 255;
        line_strip.color.g = 102 / 255;
        line_strip.color.g = 204 / 255;
        line_strip.color.a = 0.5;
        line_strip.lifetime = ros::Duration(1.0);
        line_strip.pose.orientation.x = 0;
        line_strip.pose.orientation.y = 0;
        line_strip.pose.orientation.z = 0;
        line_strip.pose.orientation.w = 1;
        for (const auto& p: xs){
            geometry_msgs::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            line_strip.points.push_back(point);
        }
        ref_msg.markers.push_back(line_strip);
    }

    _reference_pub.publish(ref_msg);
}

void PathTracker::_VisCandidate(const std::vector<geometry_msgs::Point>& pred_xs, const std::vector<double>& costs, const bool constraint){
    int T = (int)_dynamics.GetT();
    int num_path = std::min(_num_vis, _K);
    std::vector<std::vector<geometry_msgs::Point> > resized_xs;
    std::vector<geometry_msgs::Point> point_seq;

    for (const auto& xs: pred_xs){
        point_seq.push_back(xs);
        if (point_seq.size() == T){
            resized_xs.push_back(point_seq);
            point_seq.clear();
            if (resized_xs.size() == num_path){
                break;
            }
        }
    }

    visualization_msgs::MarkerArray candidate_msg;
    auto header = PathTracker::_GetHeader();
    int id = 0;
    for (const auto& xs: resized_xs){
        visualization_msgs::Marker line_strip;
        line_strip.header = header;
        line_strip.id = id++;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.scale.x = 0.01;
        line_strip.color.r = 180/255;
        line_strip.color.g = 255/255;
        line_strip.color.b = 121/255;
        line_strip.color.a = 0.5;
        line_strip.lifetime = ros::Duration(1.0);
        line_strip.pose.orientation.x = 0;
        line_strip.pose.orientation.y = 0;
        line_strip.pose.orientation.z = 0;
        line_strip.pose.orientation.w = 1;
        for (const auto& p: xs){
            geometry_msgs::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            line_strip.points.push_back(point);
        }
        candidate_msg.markers.push_back(line_strip);
    }

    if (constraint) {
        for (int n = 0; n < candidate_msg.markers.size(); n++){
            for (const auto& idx : _cost.const_indices){
                if (candidate_msg.markers.at(n).id == idx){
                    candidate_msg.markers.at(n).color.r = 1.;
                    candidate_msg.markers.at(n).color.g = 0.;
                    candidate_msg.markers.at(n).color.b = 0.;
                }
            }
        }
    }
    _candidate_pub.publish(candidate_msg);
}

void PathTracker::_VisLpSubgoal(const geometry_msgs::Point& lp_subgoal, bool marker) {
    visualization_msgs::Marker goal;
    auto header = PathTracker::_GetHeader();

    if (marker) goal.type = goal.CYLINDER;
    else goal.type = goal.CUBE;
    goal.action = goal.ADD;
    goal.header = header;
    goal.scale.x = 0.5;
    goal.scale.y = 0.5;
    goal.scale.z = 0.2;
    goal.pose.position.x = lp_subgoal.x;
    goal.pose.position.y = lp_subgoal.y;
    goal.pose.position.z = 0;
    goal.color.r = 1.0;
    goal.color.g = 0;
    goal.color.b = 0;
    goal.color.a = 1.0;

    _lp_subgoal_pub.publish(goal);
}

void PathTracker::_VisTrackerSubgoal(const geometry_msgs::Point& tracker_subgoal, bool marker) {
    visualization_msgs::Marker goal;
    auto header = PathTracker::_GetHeader();

    if (!marker) goal.type = goal.CYLINDER;
    else goal.type = goal.CUBE;
    goal.action = goal.ADD;
    goal.header = header;
    goal.scale.x = 0.5;
    goal.scale.y = 0.5;
    goal.scale.z = 0.2;
    goal.pose.position.x = tracker_subgoal.x;
    goal.pose.position.y = tracker_subgoal.y;
    goal.pose.position.z = 0;
    goal.color.r = 0;
    goal.color.g = 0;
    goal.color.b = 1.0;
    goal.color.a = 1.0;

    _tracker_subgoal_pub.publish(goal);
}

std_msgs::Header PathTracker::_GetHeader(){
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = _frame_id;
    return header;
}
