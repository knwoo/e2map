#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import LinkState
from std_msgs.msg import Bool

import numpy as np


class AgentResetNode():
    def __init__(self):
        rospy.init_node('set_link_pose_node', anonymous=True)
        rospy.Subscriber('/reset_complete_topic', Bool, self.reset_complete_callback)
        
        # default respawn location : human scenario
        self.scenario = rospy.get_param("/scenario", 'human') 

        # pos = (x,y,z,R,P,Y)
        self.scenario_to_spawn_pos = {
            'human': (-0.31, 0, 0.55, 0, 0, 0),
            'door': (-3.627866, 2.194374, 0.55, 0, 0, -1.57),
            'danger': (-2.864374, -1.756540, 0.55, 0, 0, 0)
        }
        
    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        # Calculate the quaternion components
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]

    def set_link_pose(self, link_name, pose):
        rospy.wait_for_service('/gazebo/set_link_state')
        try:
            set_link_state_service = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)

            # Set the model state
            link_state = LinkState()
            link_state.link_name = link_name

            # Set position
            link_state.pose.position.x = pose[0]
            link_state.pose.position.y = pose[1]
            link_state.pose.position.z = pose[2]

            quaternion = self.euler_to_quaternion(pose[3], pose[4], pose[5])
            
            # Set orientation (quaternion)
            link_state.pose.orientation.x = quaternion[0]
            link_state.pose.orientation.y = quaternion[1]
            link_state.pose.orientation.z = quaternion[2]
            link_state.pose.orientation.w = quaternion[3]
            
            # Set twist (velocity)
            link_state.twist.linear.x = 0.0
            link_state.twist.linear.y = 0.0
            link_state.twist.linear.z = 0.0
            link_state.twist.angular.x = 0.0
            link_state.twist.angular.y = 0.0
            link_state.twist.angular.z = 0.0

            link_state.reference_frame = 'base'

            # Call the service
            resp = set_link_state_service(link_state)
            if resp.success:
                rospy.loginfo(f"Successfully moved {link_state.link_name} to the new position.")
            else:
                rospy.logwarn(f"Failed to move {link_state.link_name}: {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def set_model_pose(self):
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

            # Set the model state
            model_state = ModelState()
            model_state.model_name = 'go1_gazebo'
            
            # Spawn pose w.r.t scenario
            pos = self.scenario_to_spawn_pos[self.scenario]
            quat = self.euler_to_quaternion(pos[3], pos[4], pos[5])
            
            # Set position
            model_state.pose.position.x = pos[0]
            model_state.pose.position.y = pos[1]
            model_state.pose.position.z = pos[2]

            # Set orientation (quaternion)
            model_state.pose.orientation.x = quat[0]
            model_state.pose.orientation.y = quat[1]
            model_state.pose.orientation.z = quat[2]
            model_state.pose.orientation.w = quat[3]

            # Set twist (velocity)
            model_state.twist.linear.x = 0.0
            model_state.twist.linear.y = 0.0
            model_state.twist.linear.z = 0.0
            model_state.twist.angular.x = 0.0
            model_state.twist.angular.y = 0.0
            model_state.twist.angular.z = 0.0

            model_state.reference_frame = 'world'

            # Call the service
            resp = set_model_state_service(model_state)
            if resp.success:
                rospy.loginfo(f"Successfully moved {model_state.model_name} to the new position.")
            else:
                rospy.logwarn(f"Failed to move {model_state.model_name}: {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def reset_complete_callback(self, msg):
        reset_complete = msg.data
        if reset_complete:
            self.set_link_pose('go1_gazebo::FL_hip', [0.188100, 0.046750, 0.0, 0.077828, 0.0, 0.0])
            self.set_link_pose('go1_gazebo::FL_thigh', [0.188101, 0.126508, 0.006213, 0.099543, 0.673361, 0.062233])
            self.set_link_pose('go1_gazebo::FL_calf', [0.054517, 0.139384, -0.159196, 0.109444, -0.777947, -0.077037])
            self.set_link_pose('go1_gazebo::FR_hip', [0.188100, -0.046750, 0.0, -0.076685, 0.0, 0.0])
            self.set_link_pose('go1_gazebo::FR_thigh', [0.188101, -0.126508, 0.006213, -0.099543, 0.673361, -0.062233])
            self.set_link_pose('go1_gazebo::FR_calf', [0.054517, -0.139384, -0.159196, -0.109444, -0.777947, 0.077037])
            self.set_link_pose('go1_gazebo::RL_hip', [-0.188100, 0.046750, 0.0, 0.069028, 0.0, 0.0])
            self.set_link_pose('go1_gazebo::RL_thigh', [-0.188101, 0.126508, 0.005515, 0.093837, 0.743721, 0.063649])
            self.set_link_pose('go1_gazebo::RL_calf', [-0.332899, 0.137328, -0.150325, 0.088228, -0.670601, -0.054963])
            self.set_link_pose('go1_gazebo::RR_hip', [-0.188100, -0.046750, 0.0, -0.069028, 0.0, 0.0])
            self.set_link_pose('go1_gazebo::RR_thigh', [-0.188101, -0.126508, 0.005515, -0.093837, 0.743721, -0.063649])
            self.set_link_pose('go1_gazebo::RR_calf', [-0.332899, -0.137328, -0.150325, -0.088228, -0.670601, 0.054963])
            self.set_model_pose()


if __name__ == '__main__':
    try:
        agent_reset_node = AgentResetNode()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():    
            rate.sleep()
        
    except rospy.ROSInterruptException:
        pass
