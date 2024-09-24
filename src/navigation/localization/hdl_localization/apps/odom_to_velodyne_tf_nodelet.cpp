#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace hdl_localization {

class OdomToVelodyneTFPublisher : public nodelet::Nodelet {
public:
    OdomToVelodyneTFPublisher() = default;

private:
    virtual void onInit() override {
        ros::NodeHandle& nh = getNodeHandle();
        private_nh = getPrivateNodeHandle();

        // initialize localization params
        nh.getParam("/odom_frame_id", odom_frame_id);
        nh.getParam("/loc_go1_topic", loc_go1_topic);
        child_frame_id = private_nh.param<std::string>("odom_child_frame_id", "velodyne");

        odom_sub_ = nh.subscribe(loc_go1_topic, 10, &OdomToVelodyneTFPublisher::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        static tf::TransformBroadcaster br;
        geometry_msgs::TransformStamped t;

        t.header.stamp = msg->header.stamp;
        t.header.frame_id = odom_frame_id;
        t.child_frame_id = child_frame_id;

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        t.transform.rotation = msg->pose.pose.orientation;

        br.sendTransform(t);
    }

    ros::NodeHandle private_nh;
    ros::Subscriber odom_sub_;
    std::string odom_frame_id;
    std::string loc_go1_topic;
    std::string child_frame_id;
};

PLUGINLIB_EXPORT_CLASS(hdl_localization::OdomToVelodyneTFPublisher, nodelet::Nodelet)

}

