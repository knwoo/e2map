#!/usr/bin/env python3
import math
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler

def publish_path():
    # Initialize the node
    rospy.init_node('path_publisher_node')
    
    # Create the publisher with topic "/actor_path" and message type "Path"
    pub = rospy.Publisher('/actor_path', Path, queue_size=1, latch=True)
    
    # Create the Path message
    path_msg = Path()
    
    # Set the header for the Path message
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"

    pose1 = PoseStamped()
    pose1.header.stamp = rospy.Time.now()
    pose1.header.frame_id = "map"
    pose1.pose.position = Point(x=3.7754316362273266, y=1.5891523126177762, z=1.0)
    path_msg.poses.append(pose1)

    pose2 = PoseStamped()
    pose2.header.stamp = rospy.Time.now()
    pose2.header.frame_id = "map"
    pose2.pose.position = Point(x=3.776812821332524, y=-0.14734944941103365, z=1.0)
    path_msg.poses.append(pose2)

    pose3 = PoseStamped()
    pose3.header.stamp = rospy.Time.now()
    pose3.header.frame_id = "map"
    pose3.pose.position = Point(x=2.9498299663034566, y=-1.2102610404607805, z=1.0)
    path_msg.poses.append(pose3)

    pose4 = PoseStamped()
    pose4.header.stamp = rospy.Time.now()
    pose4.header.frame_id = "map"
    pose4.pose.position = Point(x=0.20582302044404882, y=-1.1422488633992778, z=1.0)
    path_msg.poses.append(pose4)

    pose5 = PoseStamped()
    pose5.header.stamp = rospy.Time.now()
    pose5.header.frame_id = "map"
    pose5.pose.position = Point(x=-1.8239514761703903, y=-1.617291497885034, z=1.0)
    path_msg.poses.append(pose5)

    pose6 = PoseStamped()
    pose6.header.stamp = rospy.Time.now()
    pose6.header.frame_id = "map"
    pose6.pose.position = Point(x=-4.161645409685235, y=-2.1047922458612858, z=1.0)
    path_msg.poses.append(pose6)

    rospy.sleep(0.5)
    pub.publish(path_msg)
    rospy.loginfo("Published path: %s", path_msg)
    rospy.sleep(2)

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
