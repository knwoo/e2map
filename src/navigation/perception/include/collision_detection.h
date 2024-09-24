#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <cmath>
#include <vector>
// #include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class CollisionDetection
{
    public:
        CollisionDetection();
        void Run();

    private:
        void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void FilterPointCloud(const sensor_msgs::PointCloud2& input_cloud, sensor_msgs::PointCloud2& output_cloud, std_msgs::Bool& collision_msg);
        void ParseStringToDoubleVector(const std::string& str, std::vector<double>& vec);
        void ParseStringToIntVector(const std::string& str, std::vector<int>& vec);
        void GetParameters();

    private:
        ros::NodeHandle nh;
        ros::Publisher is_collision_pub;
        ros::Subscriber point_cloud_sub;

        string collision_detection_topic;

        std::vector<double> angle_thresholds;
        std::vector<int> channel_thresholds;
        int pc_cnt_thres;

        bool collision_message_published;
};