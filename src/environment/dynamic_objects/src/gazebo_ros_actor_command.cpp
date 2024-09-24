#include <gazebo_ros_actor_command.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <functional>

#include "gazebo/physics/physics.hh"
#include <ignition/math.hh>

using namespace gazebo;

#define _USE_MATH_DEFINES
#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
GazeboRosActorCommand::GazeboRosActorCommand() {
}

GazeboRosActorCommand::~GazeboRosActorCommand() {
  this->vel_queue_.clear();
  this->vel_queue_.disable();
  this->velCallbackQueueThread_.join();

  // Added for path
  this->path_queue_.clear();
  this->path_queue_.disable();
  this->pathCallbackQueueThread_.join();

  this->reset_queue_.clear();  // Reset queue clear
  this->reset_queue_.disable();  // Reset queue disable
  this->resetCallbackQueueThread_.join();  // Join reset thread

  this->ros_node_->shutdown();
  delete this->ros_node_;
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  
  this->ros_node_ = new ros::NodeHandle();

  // Set default values for parameters
  this->ros_node_->getParam("/follow_mode", this->follow_mode_);
  this->ros_node_->getParam("/vel_topic", this->vel_topic_);
  this->ros_node_->getParam("/path_topic", this->path_topic_);
  this->ros_node_->getParam("/linear_velocity", this->lin_velocity_);
  this->ros_node_->getParam("/reset_topic", this->reset_topic_);
  this->lin_tolerance_ = 0.1;
  this->ang_tolerance_ = IGN_DTOR(5);
  this->ang_velocity_ = IGN_DTOR(40);
  this->animation_factor_ = 4.0;

  this->pose_pub_ = this->ros_node_->advertise<geometry_msgs::PoseStamped>("/actor_pose", 10);

  // Override default parameter values with values from SDF
  if (this->follow_mode_.empty()) {
    this->follow_mode_ = "velocity";
  }
  if (this->vel_topic_.empty()) {
    this->vel_topic_ = "/cmd_vel";
  }
  if (this->path_topic_.empty()) {
    this->path_topic_ = "/cmd_path";
  }
  if (_sdf->HasElement("linear_tolerance")) {
    this->lin_tolerance_ = _sdf->Get<double>("linear_tolerance");
  }
  if (_sdf->HasElement("angular_tolerance")) {
    this->ang_tolerance_ = _sdf->Get<double>("angular_tolerance");
  }
  if (_sdf->HasElement("angular_velocity")) {
    this->ang_velocity_ = _sdf->Get<double>("angular_velocity");
  }
  if (_sdf->HasElement("animation_factor")) {
    this->animation_factor_ = _sdf->Get<double>("animation_factor");
  }
  if (_sdf->HasElement("default_rotation")) {
    this->default_rotation_ = _sdf->Get<double>("default_rotation");
  }
  
  // Check if ROS node for Gazebo has been initialized
  if (!ros::isInitialized()) {
    ROS_FATAL_STREAM_NAMED("actor", "A ROS node for Gazebo has not been "
    << "initialized, unable to load plugin. Load the Gazebo system plugin "
    << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Set variables
  this->sdf_ = _sdf;
  this->actor_ = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world_ = this->actor_->GetWorld();
  this->Reset();

  // Subscribe to the velocity commands
  ros::SubscribeOptions vel_so =
    ros::SubscribeOptions::create<geometry_msgs::Twist>(
      vel_topic_,
      1,
      boost::bind(&GazeboRosActorCommand::VelCallback, this, _1),
      ros::VoidPtr(),
      &vel_queue_);
  this->vel_sub_ = ros_node_->subscribe(vel_so);

  // Create a thread for the velocity callback queue
  this->velCallbackQueueThread_ =
      boost::thread(boost::bind(&GazeboRosActorCommand::VelQueueThread, this));

  // Subscribe to the path commands
  ros::SubscribeOptions path_so =
    ros::SubscribeOptions::create<nav_msgs::Path>(
        path_topic_,
        1,
        boost::bind(&GazeboRosActorCommand::PathCallback, this, _1),
        ros::VoidPtr(),
        &path_queue_);
  this->path_sub_ = ros_node_->subscribe(path_so);

  // Create a thread for the path callback queue
  this->pathCallbackQueueThread_ =
      boost::thread(boost::bind(&GazeboRosActorCommand::PathQueueThread, this));

  // Subscribe to the reset command
  ros::SubscribeOptions reset_so =
    ros::SubscribeOptions::create<std_msgs::Bool>(
      this->reset_topic_,
      1,
      boost::bind(&GazeboRosActorCommand::ResetCallback, this, _1),
      ros::VoidPtr(),
      &reset_queue_);
  this->reset_sub_ = ros_node_->subscribe(reset_so);

  // Create a thread for the reset callback queue
  this->resetCallbackQueueThread_ =
      boost::thread(boost::bind(&GazeboRosActorCommand::ResetQueueThread, this));

  // Connect the OnUpdate function to the WorldUpdateBegin event.
  this->connections_.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboRosActorCommand::OnUpdate, this, std::placeholders::_1)));
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::Reset() {
  // Reset last update time and target pose index
  this->last_update_ = 0;
  this->idx_ = 0;
  // Initialize target poses vector with origin
  this->target_poses_.clear();  // Clear previous targets
  initial_pose_ = this->actor_->WorldPose();
  this->target_pose_ = ignition::math::Vector3d(initial_pose_.Pos().X(), 
                                                initial_pose_.Pos().Y(), 
                                                initial_pose_.Rot().Euler().Z());

  // Check if the walking animation exists in the actor's skeleton animations
  auto skelAnims = this->actor_->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end()) {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  } else {
    // Create custom trajectory
    this->trajectoryInfo_.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo_->type = WALKING_ANIMATION;
    this->trajectoryInfo_->duration = 1.0;

    // Set the actor's trajectory to the custom trajectory
    this->actor_->SetCustomTrajectory(this->trajectoryInfo_);
  }
}

void GazeboRosActorCommand::ResetCallback(const std_msgs::Bool::ConstPtr &msg) {
  if (msg->data) {
    this->ResetActorPos();
  }
}

void GazeboRosActorCommand::VelCallback(const geometry_msgs::Twist::ConstPtr &msg) {
  ignition::math::Vector3d vel_cmd;
  vel_cmd.X() = msg->linear.x;
  vel_cmd.Z() = msg->angular.z;
  this->cmd_queue_.push(vel_cmd);
}

void GazeboRosActorCommand::PathCallback(const nav_msgs::Path::ConstPtr &msg) {
  this->target_poses_.clear();
  this->idx_ = 0;
  this->target_pose_ = ignition::math::Vector3d(initial_pose_.Pos().X(), 
                                                initial_pose_.Pos().Y(), 
                                                initial_pose_.Rot().Euler().Z());
                                                
  // Extract the poses from the Path message
  const std::vector<geometry_msgs::PoseStamped>& poses = msg->poses;

  // Extract the x, y, and yaw from each pose and store it as a target
  for (size_t i = 0; i < poses.size(); ++i) {
    const geometry_msgs::Pose& pose = poses[i].pose;
    const double x = pose.position.x;
    const double y = pose.position.y;

    // Convert quaternion to Euler angles
    tf2::Quaternion quat(pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    this->target_poses_.push_back(ignition::math::Vector3d(x, y, yaw));
  }
}

/////////////////////////////////////////////////
void GazeboRosActorCommand::OnUpdate(const common::UpdateInfo &_info) {
  // Time delta
  double dt = (_info.simTime - this->last_update_).Double();
  ignition::math::Pose3d pose = this->actor_->WorldPose();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  // Check if the follow mode is "path" and if there are any targets in target_poses_
  if (this->follow_mode_ == "path" && this->target_poses_.size() > 1) {
    ignition::math::Vector2d target_pos_2d(this->target_pose_.X(),
                                           this->target_pose_.Y());
    ignition::math::Vector2d current_pos_2d(pose.Pos().X(),
                                            pose.Pos().Y());
    ignition::math::Vector2d pos = target_pos_2d - current_pos_2d;
    double distance = pos.Length();

    // Check if actor has reached current target position
    if (distance < this->lin_tolerance_) {
      // If there are more targets, choose new target
      if (this->idx_ < this->target_poses_.size() - 1) {
        this->ChooseNewTarget();
        pos.X() = this->target_pose_.X() - pose.Pos().X();
        pos.Y() = this->target_pose_.Y() - pose.Pos().Y();
      } else {
        // All targets have been accomplished, stop moving
        pos.X() = 0;
        pos.Y() = 0;
      }
    }

    // Normalize the direction vector
    if (pos.Length() != 0) {
      pos = pos / pos.Length();
    }

    int rot_sign = 1;
    // Calculate the angular displacement required based on the direction
    // vector towards the current target position
    ignition::math::Angle yaw(0);
    if (pos.Length() != 0) {
      yaw = atan2(pos.Y(), pos.X()) + default_rotation_ - rpy.Z();
      yaw.Normalize();
    }
    if (yaw < 0)
      rot_sign = -1;
    // Check if required angular displacement is greater than tolerance
    if (std::abs(yaw.Radian()) > this->ang_tolerance_) {
      pose.Rot() = ignition::math::Quaterniond(default_rotation_, 0, rpy.Z() +
                                               rot_sign * this->ang_velocity_ * dt);
    } else {
      // Move towards the target position
      pose.Pos().X() += pos.X() * this->lin_velocity_ * dt;
      pose.Pos().Y() += pos.Y() * this->lin_velocity_ * dt;

      pose.Rot() = ignition::math::Quaterniond(
          default_rotation_, 0, rpy.Z() + yaw.Radian());
    }
  }
  else if (this->follow_mode_ == "velocity") {
    if (!this->cmd_queue_.empty()) {
      this->target_vel_.Pos().X() = this->cmd_queue_.front().X();
      this->target_vel_.Rot() = ignition::math::Quaterniond(
          0, 0, this->cmd_queue_.front().Z());
      this->cmd_queue_.pop();
    }

    pose.Pos().X() += this->target_vel_.Pos().X() *
                      cos(pose.Rot().Euler().Z() - default_rotation_) * dt;
    pose.Pos().Y() += this->target_vel_.Pos().X() *
                      sin(pose.Rot().Euler().Z() - default_rotation_) * dt;

    pose.Rot() = ignition::math::Quaterniond(
        default_rotation_, 0, rpy.Z() + this->target_vel_.Rot().Euler().Z() * dt);
  }

  // Create a PoseStamped message
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.header.frame_id = "world";

  // Convert Gazebo pose to geometry_msgs::Pose
  pose_msg.pose.position.x = pose.Pos().X();
  pose_msg.pose.position.y = pose.Pos().Y();
  pose_msg.pose.position.z = pose.Pos().Z();

  pose_msg.pose.orientation.x = pose.Rot().X();
  pose_msg.pose.orientation.y = pose.Rot().Y();
  pose_msg.pose.orientation.z = pose.Rot().Z();
  pose_msg.pose.orientation.w = pose.Rot().W();

  // Publish the pose
  this->pose_pub_.publish(pose_msg);

  // Distance traveled is used to coordinate motion with the walking animation
  auto displacement = pose.Pos() - this->actor_->WorldPose().Pos();
  double distanceTraveled = displacement.Length();

  this->actor_->SetWorldPose(pose, false, false);
  this->actor_->SetScriptTime(
  this->actor_->ScriptTime() + (distanceTraveled * this->animation_factor_));
  this->last_update_ = _info.simTime;
}

void GazeboRosActorCommand::ChooseNewTarget() {
  this->idx_++;

  // Set next target
  this->target_pose_ = this->target_poses_.at(this->idx_);
}

void GazeboRosActorCommand::VelQueueThread() {
  static const double timeout = 0.01;

  while (this->ros_node_->ok())
    this->vel_queue_.callAvailable(ros::WallDuration(timeout));
}

void GazeboRosActorCommand::PathQueueThread() {
  static const double timeout = 0.01;

  while (this->ros_node_->ok())
    this->path_queue_.callAvailable(ros::WallDuration(timeout));
}

void GazeboRosActorCommand::ResetQueueThread() {
  static const double timeout = 0.01;

  while (this->ros_node_->ok())
    this->reset_queue_.callAvailable(ros::WallDuration(timeout));
}

void GazeboRosActorCommand::ResetActorPos() {
  this->last_update_ = 0;
  this->idx_ = 0;
  this->target_poses_.clear();

  ignition::math::Pose3d pose = initial_pose_;

  this->actor_->SetWorldPose(pose, false, false);

  this->target_vel_.Pos().X(0);
  this->target_vel_.Pos().Y(0);
  this->target_vel_.Pos().Z(0);
  this->target_vel_.Rot() = ignition::math::Quaterniond(0, 0, 0);

  this->actor_->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
  this->actor_->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosActorCommand)