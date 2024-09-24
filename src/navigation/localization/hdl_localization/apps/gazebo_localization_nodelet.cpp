#include <iostream>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <pluginlib/class_list_macros.h>

namespace hdl_localization {

    class GazeboLocalizationNodelet : public nodelet::Nodelet {
    public:
        GazeboLocalizationNodelet() {}

        ~GazeboLocalizationNodelet() {}

        void onInit() override {
            nh = getNodeHandle();
            private_nh = getPrivateNodeHandle();

            initialize_params();

            pose_pub = nh.advertise<nav_msgs::Odometry>(localization_topic, 5, false);
            gazebo_model_state_sub = nh.subscribe(gazebo_model_states_topic, 8,
                                                  &GazeboLocalizationNodelet::gazebo_model_state_callback,
                                                  this); // TODO queue
        }

    private:
        void initialize_params() {
            odom_child_frame_id = private_nh.param<std::string>("odom_child_frame_id", "base_link");
            nh.getParam("/localization_topic", localization_topic);
            nh.getParam("/gazebo_model_name", gazebo_model_name);
            nh.getParam("/gazebo_model_states_topic", gazebo_model_states_topic);
        }

        void gazebo_model_state_callback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
            ros::Rate gazebo_rate(10);
            NODELET_INFO("odometry received!!");

            // Find the index of the specified model
            auto it = std::find(msg->name.begin(), msg->name.end(), gazebo_model_name);
            if (it == msg->name.end()) {
                ROS_WARN("Model %s not found in %s", gazebo_model_name.c_str(), gazebo_model_states_topic.c_str());
                return;
            }

            int index = std::distance(msg->name.begin(), it);

            nav_msgs::Odometry odom;
            odom.header.frame_id = "map";
            odom.child_frame_id = odom_child_frame_id;

            odom.pose.pose = msg->pose[index];
            odom.twist.twist = msg->twist[index];

            pose_pub.publish(odom);
            gazebo_rate.sleep();
        }

    private:
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        std::string odom_child_frame_id;
        std::string localization_topic;
        std::string gazebo_model_name;
        std::string gazebo_model_states_topic;

        ros::Publisher pose_pub;
        ros::Subscriber gazebo_model_state_sub;
    };
}

PLUGINLIB_EXPORT_CLASS(hdl_localization::GazeboLocalizationNodelet, nodelet::Nodelet)