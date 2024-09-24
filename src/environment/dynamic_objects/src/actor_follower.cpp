#include "actor_follower.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "actor_follower");

    ActorFollower actor_follower;
    actor_follower.Run();

    return 0;
}

ActorFollower::ActorFollower() : has_published_(false) {
    nh_.getParam(ros::this_node::getName() + "/follow_distance", follow_distance_);
    nh_.getParam(ros::this_node::getName() + "/actor_speed", actor_speed_);
    nh_.getParam(ros::this_node::getName() + "/x_thr", x_threshold_);
    nh_.getParam(ros::this_node::getName() + "/y_thr", y_threshold_);
    nh_.getParam("follow_mode", follow_mode_);
    nh_.getParam("/reset_topic", reset_topic_);

    odom_sub_ = nh_.subscribe("/gt_pose", 10, &ActorFollower::OdomCallback, this);
    actor_pose_sub_ = nh_.subscribe("/actor_pose", 10, &ActorFollower::ActorPoseCallback, this);
    reset_sub_ = nh_.subscribe(reset_topic_, 10, &ActorFollower::ResetCallback, this);

    if (follow_mode_ == "path") {
        actor_path_pub_ = nh_.advertise<nav_msgs::Path>("/actor_path", 10);
    } else {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/actor_vel", 10);
    }
}

ActorFollower::~ActorFollower() {}

void ActorFollower::Run() {
    ros::spin();
}

void ActorFollower::OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    robot_pose_ = msg->pose.pose;
}

void ActorFollower::ResetCallback(const std_msgs::Bool::ConstPtr &msg) {
    if (msg->data) {
        has_published_ = false;
    }
}

void ActorFollower::ActorPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    actor_pose_ = msg->pose;

    double distance = CalculateDistance();
    std::cout << "distance : " << distance << std::endl;

    if (!has_published_) {
        if (follow_mode_ == "path") {
            if (robot_pose_.position.x > x_threshold_ && robot_pose_.position.y < y_threshold_ ) {
                PublishPath();
                ros::Duration(3).sleep();
                has_published_ = true;
            }
        } else {
            if (distance < follow_distance_) {
                geometry_msgs::Twist twist_msg;
                twist_msg.linear.x = actor_speed_;
                twist_msg.angular.z = 0.0;

                cmd_vel_pub_.publish(twist_msg);
                ros::Duration(3).sleep();
                has_published_ = true;
            }
        }
    }
    ros::Duration(0.2).sleep();
}

double ActorFollower::CalculateDistance() {
    double dx = robot_pose_.position.x - actor_pose_.position.x;
    double dy = robot_pose_.position.y - actor_pose_.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

void ActorFollower::PublishPath() {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    geometry_msgs::PoseStamped pose1;
    pose1.header.stamp = ros::Time::now();
    pose1.header.frame_id = "map";
    pose1.pose.position.x = 3.7754316362273266;
    pose1.pose.position.y = 1.5891523126177762;
    pose1.pose.position.z = 1.0;
    path_msg.poses.push_back(pose1);

    geometry_msgs::PoseStamped pose2;
    pose2.header.stamp = ros::Time::now();
    pose2.header.frame_id = "map";
    pose2.pose.position.x = 3.776812821332524;
    pose2.pose.position.y = -0.14734944941103365;
    pose2.pose.position.z = 1.0;
    path_msg.poses.push_back(pose2);

    geometry_msgs::PoseStamped pose3;
    pose3.header.stamp = ros::Time::now();
    pose3.header.frame_id = "map";
    pose3.pose.position.x = 2.9498299663034566;
    pose3.pose.position.y = -1.2102610404607805;
    pose3.pose.position.z = 1.0;
    path_msg.poses.push_back(pose3);

    geometry_msgs::PoseStamped pose4;
    pose4.header.stamp = ros::Time::now();
    pose4.header.frame_id = "map";
    pose4.pose.position.x = 0.20582302044404882;
    pose4.pose.position.y = -1.1422488633992778;
    pose4.pose.position.z = 1.0;
    path_msg.poses.push_back(pose4);

    geometry_msgs::PoseStamped pose5;
    pose5.header.stamp = ros::Time::now();
    pose5.header.frame_id = "map";
    pose5.pose.position.x = -1.8239514761703903;
    pose5.pose.position.y = -1.617291497885034;
    pose5.pose.position.z = 1.0;
    path_msg.poses.push_back(pose5);

    geometry_msgs::PoseStamped pose6;
    pose6.header.stamp = ros::Time::now();
    pose6.header.frame_id = "map";
    pose6.pose.position.x = -4.161645409685235;
    pose6.pose.position.y = -2.1047922458612858;
    pose6.pose.position.z = 1.0;
    path_msg.poses.push_back(pose6);

    ros::Duration(0.5).sleep();
    actor_path_pub_.publish(path_msg);
    ROS_INFO("Published path.");
}