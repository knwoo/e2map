#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

class ActorFollower {
public:
    ActorFollower();
    ~ActorFollower();

    void Run();

private:
    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void ResetCallback(const std_msgs::Bool::ConstPtr &msg);
    void ActorPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    double CalculateDistance();
    void PublishPath();

    ros::NodeHandle nh_;

    std::string distance_;
    std::string follow_mode_;
    std::string reset_topic_;

    ros::Subscriber reset_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber actor_pose_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher actor_path_pub_;

    geometry_msgs::Pose robot_pose_;
    geometry_msgs::Pose actor_pose_;
    bool has_published_;
    double follow_distance_;
    double actor_speed_;
    double x_threshold_;
    double y_threshold_;
};