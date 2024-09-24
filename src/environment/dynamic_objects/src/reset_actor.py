#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

class ResetNode():
    def __init__(self):
        self.update_finished_topic_name = rospy.get_param('/update_finished_topic')
        self.success_topic_name = rospy.get_param('/success_topic')
        self.reset_topic_name = rospy.get_param('/reset_topic')
        self.update_finished_sub = rospy.Subscriber(self.update_finished_topic_name, Bool, self.update_finished_callback)
        self.success_sub = rospy.Subscriber(self.success_topic_name, Bool, self.success_callback)
        self.reset_pub = rospy.Publisher(self.reset_topic_name, Bool, queue_size=10)
        self.update_finished = False
        self.success = False

    def update_finished_callback(self, msg):
        self.update_finished = msg.data
        
    def success_callback(self, msg):
        self.success = msg.data

    def reset_publisher(self):
        if self.update_finished or self.success:
            reset_msg = Bool()
            reset_msg.data = True
            self.reset_pub.publish(reset_msg)
            rospy.loginfo("Reset signal sent!")
            self.update_finished = False
            self.success = False

if __name__ == '__main__':
    try:
        rospy.init_node('reset_publisher_node', anonymous=True)
        reset_node = ResetNode()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():    
            reset_node.reset_publisher()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass