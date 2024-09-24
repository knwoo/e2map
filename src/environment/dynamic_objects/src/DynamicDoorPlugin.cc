#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ignition/math.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <std_msgs/UInt32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#define DEFAULT_OPEN_VEL   1.0
#define DEFAULT_CLOSE_VEL -1.0

#define DIRECTION_FLIP_CLOCKWISE "clockwise"
#define DIRECTION_FLIP_COUNTER_CLOCKWISE "counter_clockwise"

namespace gazebo
{
  class DoorPlugin : public ModelPlugin
  {
  private:
    physics::ModelPtr model_;
    physics::LinkPtr doorLink_;

    ignition::math::Vector3d cmd_vel_;

    bool isActive_;
    std::string door_direction_;
    double threshold_distance_;
    
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;

    event::ConnectionPtr updateConnection_;

  public:
    DoorPlugin()
    {
        std::string name = "door_plugin_node";
        int argc = 0;
        ros::init(argc, nullptr, name);
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
        model_ = _parent;
        doorLink_ = model_->GetLink("door");

        if (!_sdf->HasElement("door_direction")) {
            ROS_WARN("Door direction not specified. Defaulting to 'clockwise'");
            door_direction_ = DIRECTION_FLIP_CLOCKWISE;
        } else {
            door_direction_ = _sdf->GetElement("door_direction")->Get<std::string>();
        }

        if (!nh_.getParam("threshold_distance", threshold_distance_)) {
            ROS_WARN("Threshold distance not specified on parameter server. Defaulting to 1.0");
            threshold_distance_ = 1.0;
        }

        odom_sub_ = nh_.subscribe<nav_msgs::Odometry>("/gt_pose", 100, &DoorPlugin::odom_cb, this);

        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&DoorPlugin::OnUpdate, this));

        isActive_ = false;
    }

    void OnUpdate()
    {
        ros::spinOnce();
        updateLinkVel();
    }

  private:
    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ignition::math::Vector3d door_position = model_->WorldPose().Pos();
        ignition::math::Vector3d robot_position(msg->pose.pose.position.x, msg->pose.pose.position.y, 0);

        double distance = door_position.Distance(robot_position);

        if (distance < threshold_distance_) {
            if (!isActive_) {  
                isActive_ = true;
                setAngularVel(DEFAULT_OPEN_VEL);
                ROS_INFO("Door '%s' - Opening. Distance: [%f]", model_->GetName().c_str(), distance);
            }
        } else {
            if (isActive_) { 
                isActive_ = false;
                setAngularVel(DEFAULT_CLOSE_VEL);
                ROS_INFO("Door '%s' - Closing. Distance: [%f]", model_->GetName().c_str(), distance);
            }
        }
    }

    void updateLinkVel()
    {
        if (isActive_) {
            doorLink_->SetAngularVel(cmd_vel_);
        } else {
            doorLink_->SetAngularVel(cmd_vel_);
        }
    }

    void setAngularVel(float rot_z)
    {
        cmd_vel_ = ignition::math::Vector3d();

        if (door_direction_ == DIRECTION_FLIP_CLOCKWISE) {
            cmd_vel_.Z() = rot_z;
        } else {
            cmd_vel_.Z() = -rot_z; 
        }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(DoorPlugin)
}