#include "collision_detection.h"

CollisionDetection::CollisionDetection()
{
    ros::NodeHandle nh("~");
    GetParameters();
    is_collision_pub = nh.advertise<std_msgs::Bool>(collision_detection_topic, 10);
    point_cloud_sub = nh.subscribe("/velodyne_points", 10, &CollisionDetection::PointCloudCallback, this);
    collision_message_published = false;
}

void CollisionDetection::GetParameters() 
{
    string node_name = ros::this_node::getName();
    nh.param<std::string>("/collision_detection_topic", collision_detection_topic, "/is_collision");
    std::string angle_thresholds_str;
    nh.param<std::string>(node_name + "/angle_thresholds", angle_thresholds_str, "-1.5,1.5");
    ParseStringToDoubleVector(angle_thresholds_str, angle_thresholds);
    std::string channel_thresholds_str;
    nh.param<std::string>(node_name + "/channel_thresholds", channel_thresholds_str, "7,9");
    ParseStringToIntVector(channel_thresholds_str, channel_thresholds);
    nh.param<int>(node_name + "/pc_cnt_thres", pc_cnt_thres, 1);
}

void CollisionDetection::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if(!collision_message_published)
    {
        sensor_msgs::PointCloud2 input_cloud = *msg;
        std_msgs::Bool collision_msg;

        sensor_msgs::PointCloud2Iterator<float> iter_x(input_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(input_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(input_cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(input_cloud, "intensity");
        sensor_msgs::PointCloud2Iterator<uint16_t> iter_channel(input_cloud, "ring");
        double dist_avg = 0.0;
        int count = 0;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_channel)
        {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            uint16_t ch = *iter_channel;

            // Calculate angle
            double angle = atan2(y, x) * 180.0 / M_PI;

            if (angle_thresholds[0] <= angle && angle <= angle_thresholds[1] &&
                channel_thresholds[0] <= ch && ch <= channel_thresholds[1])
            {
                ++count;
            }
        }
        collision_msg.data = (count <= pc_cnt_thres);
        is_collision_pub.publish(collision_msg);
        if(collision_msg.data) {
            collision_message_published = true;
        }
    }
}

void CollisionDetection::ParseStringToDoubleVector(const std::string& str, std::vector<double>& vec)
{
    std::stringstream ss(str);
    double value;
    while (ss >> value)
    {
        vec.push_back(value);
        if (ss.peek() == ',')
            ss.ignore();
    }
}

void CollisionDetection::ParseStringToIntVector(const std::string& str, std::vector<int>& vec)
{
    std::stringstream ss(str);
    int value;
    while (ss >> value)
    {
        vec.push_back(value);
        if (ss.peek() == ',')
            ss.ignore();
    }
}

void CollisionDetection::Run()
{
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_detection");
    ros::NodeHandle nh;

    CollisionDetection collision_detector;

    collision_detector.Run();
    
    return 0;
}