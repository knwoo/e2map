#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class CollisionDetectionGazebo
{
    public:
        CollisionDetectionGazebo();
        void Run();

    private:
        void ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
        void LinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg);
        void ResetCallback(const std_msgs::Bool::ConstPtr& msg);
        void ParseStringToStringVector(const std::string& str, std::vector<string>& vec);
        void GetParameters();

    private:
        ros::NodeHandle nh;
        ros::Publisher is_collision_pub;
        ros::Subscriber model_sub;
        ros::Subscriber link_sub;
        std_msgs::Bool bool_msg;

        string gazebo_model_states_topic;
        string gazebo_link_states_topic;
        string collision_detection_topic;

        string go1_model_name;
        string go1_fl_hip_name;
        string go1_fr_hip_name;
        vector<string> actor_model_names;

        float dist_thres;
        bool is_collision;
        bool collision_message_published;

        float pub_freq;

        bool is_gazebo;
        Vector3d go1_position;
};