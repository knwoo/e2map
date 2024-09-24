#include "collision_detection_gazebo.h"

CollisionDetectionGazebo::CollisionDetectionGazebo()
{
    // Initialize the ROS node
    ros::NodeHandle nh("~");

    GetParameters();

    // Setup the publisher for odometry data
    is_collision_pub = nh.advertise<std_msgs::Bool>(collision_detection_topic, 10);

    // Setup the subscriber for the gazebo model states
    model_sub = nh.subscribe(gazebo_model_states_topic, 10, &CollisionDetectionGazebo::ModelStatesCallback, this);
    link_sub = nh.subscribe(gazebo_link_states_topic, 10, &CollisionDetectionGazebo::LinkStatesCallback, this);
    is_collision = false;
    collision_message_published = false;
}

void CollisionDetectionGazebo::GetParameters() 
{
    string node_name = ros::this_node::getName();
    nh.getParam("/gazebo_model_states_topic", gazebo_model_states_topic);
    nh.getParam("/gazebo_link_states_topic", gazebo_link_states_topic);
    nh.getParam("/collision_detection_topic", collision_detection_topic);
    nh.getParam("/gazebo_model_name", go1_model_name);
    go1_fl_hip_name = go1_model_name + "::FL_hip";
    go1_fr_hip_name = go1_model_name + "::FR_hip";
    std::string actor_model_names_str;
    nh.param<std::string>(node_name + "/actor_model_names", actor_model_names_str, "my_actor,door_1");
    ParseStringToStringVector(actor_model_names_str, actor_model_names);
    nh.getParam(node_name + "/pub_freq", pub_freq);
    nh.getParam(node_name + "/dist_thres", dist_thres);
}

void CollisionDetectionGazebo::ModelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    is_collision = false;

    for(int i=0; i<actor_model_names.size(); i++) {
        string actor_model_name = actor_model_names[i];
        auto actor_it = std::find(msg->name.begin(), msg->name.end(), actor_model_name);
        if(!(actor_it == msg->name.end())) {
            int actor_index = std::distance(msg->name.begin(), actor_it);
            Vector3d actor_position(msg->pose[actor_index].position.x,
                                    msg->pose[actor_index].position.y,
                                    msg->pose[actor_index].position.z);
            double distance = (go1_position.head(2) - actor_position.head(2)).norm();
            if(distance <= dist_thres) {
                is_collision = true;
                break;
            } else collision_message_published = false;
        }
    }
}

void CollisionDetectionGazebo::LinkStatesCallback(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    is_collision = false;
    
    auto go1_fl_it = std::find(msg->name.begin(), msg->name.end(), go1_fl_hip_name);
    if (go1_fl_it == msg->name.end())
    {
        return;
    }
    int go1_fl_index = std::distance(msg->name.begin(), go1_fl_it);
    Vector3d go1_fl_position(msg->pose[go1_fl_index].position.x,
                        msg->pose[go1_fl_index].position.y,
                        msg->pose[go1_fl_index].position.z);

    auto go1_fr_it = std::find(msg->name.begin(), msg->name.end(), go1_fr_hip_name);
    if (go1_fr_it == msg->name.end())
    {
        return;
    }
    int go1_fr_index = std::distance(msg->name.begin(), go1_fr_it);
    Vector3d go1_fr_position(msg->pose[go1_fr_index].position.x,
                        msg->pose[go1_fr_index].position.y,
                        msg->pose[go1_fr_index].position.z);

    go1_position = (go1_fl_position + go1_fr_position) / 2.0;

    // Find the index of the specified model
    for(int i=0; i<actor_model_names.size(); i++) {
        string actor_link_name = actor_model_names[i];
        auto actor_it = std::find(msg->name.begin(), msg->name.end(), actor_link_name);
        if(!(actor_it == msg->name.end())) {
            int actor_index = std::distance(msg->name.begin(), actor_it);
            Vector3d actor_position(msg->pose[actor_index].position.x,
                                    msg->pose[actor_index].position.y,
                                    msg->pose[actor_index].position.z);
            double distance = (go1_position.head(2) - actor_position.head(2)).norm();
            if(distance <= dist_thres) {
                is_collision = true;
                break;
            } else collision_message_published = false;
        }
    }
}

void CollisionDetectionGazebo::ParseStringToStringVector(const std::string& str, std::vector<string>& vec)
{  
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, ',')) {
        vec.push_back(token);
    }
}

void CollisionDetectionGazebo::Run()
{
    ros::Rate rate(pub_freq);
    while (ros::ok()){
        if(!collision_message_published)
        {
            bool_msg.data = is_collision;
            is_collision_pub.publish(bool_msg);
            rate.sleep();
            if(is_collision)
            {
                collision_message_published = true;
            }
        }
        ros::spinOnce();
    }

    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_detection");

    CollisionDetectionGazebo collision_detection;

    collision_detection.Run();

    return 0;
}