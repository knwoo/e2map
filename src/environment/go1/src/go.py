#!/usr/bin/env python

import sys
import os
import time
import numpy as np
import logging 
import datetime

current_path = os.path.dirname(os.path.abspath(__file__))
ucl_path = os.path.join(current_path)
sys.path.append(ucl_path)

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from ucl.common import byte_print, decode_version, decode_sn, getVoltage, pretty_print_obj, lib_version
from ucl.highCmd import highCmd
from ucl.highState import highState
from ucl.lowCmd import lowCmd
from ucl.unitreeConnection import unitreeConnection, HIGH_WIFI_DEFAULTS, HIGH_WIRED_DEFAULTS
from ucl.enums import MotorModeHigh, GaitType
from ucl.complex import motorCmd

now = datetime.datetime.now()
timestamp = now.strftime("%Y%m%d_%H%M%S")

class Go1():
    def __init__(self,loc_pub, is_gazebo):
        self.current_pos = np.array([0,0,0])
        self.current_rpy = np.array([0,0,0])
        self.current_quaternion = np.array([0,0,0,0])
        self.current_vel = np.array([0,0,0])
        self.current_yawSpeed = 0.0

        self.calib_pos = np.array([0,0,0])
        self.calib_rpy = np.array([0,0,0])
        self.calib_quaternion = np.array([0,0,0,0])
        self.goal_pos = np.array([0,0,0])

        self.count = 0
        self.init_go1()
        self.is_gazebo = is_gazebo
        if not self.is_gazebo:
            self.loc_pub = loc_pub
            self.pub()

    def init_go1(self):
        self.conn = unitreeConnection(HIGH_WIRED_DEFAULTS)
        self.conn.startRecv()

        hcmd = highCmd()
        hstate = highState()

        cmd_bytes = hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        data = self.conn.getData()
        time.sleep(0.5)
        for paket in data:
            hstate.parseData(paket)
            self.calib_pos = hstate.position
            self.calib_rpy = hstate.imu.rpy
            self.calib_quaternion = hstate.imu.quaternion

            calib_str = f'CLIB:{self.calib_pos}\t{self.calib_rpy}\t{self.calib_quaternion}\t'
            rospy.loginfo(calib_str)

    def get_state(self):
        hcmd = highCmd()
        hstate = highState()

        cmd_bytes = hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        data = self.conn.getData()

        for paket in data:
            hstate.parseData(paket)
            self.current_pos = hstate.position - self.calib_pos
            self.current_yawSpeed = hstate.yawSpeed
            self.current_rpy = hstate.imu.rpy - self.calib_rpy
            self.current_vel = hstate.velocity
            self.current_quaternion = hstate.imu.quaternion - self.calib_quaternion
        
    def cmd_vel_callback(self, msg):
        if not self.is_gazebo:
            self.pub()

        vel = msg.linear
        angular = msg.angular

        # if vel.x == 0:
        #     self.count += 1
        # else: 
        #     self.count = 0
        # if vel.y == 0:
        #     self.count += 1
        # else: 
        #     self.count = 0

        # if self.count > 3:
        #     vel.x = 0   
        #     vel.y = 0
        #     self.count = 0
        
        hcmd = highCmd()
        hcmd.mode = MotorModeHigh.VEL_WALK
        hcmd.gaitType = GaitType.TROT
        
        if (angular.z > 1):
            angular.z = 1
        elif (angular.z < -1):
            angular.z = -1
        
        if (vel.x > 0.5):
            vel.x = 0.5
        elif (vel.x < -0.5):
            vel.x = -0.5

        hcmd.velocity = [vel.x, vel.y] # -1  ~ +1
        hcmd.yawSpeed = angular.z

        cmd_bytes = hcmd.buildCmd(debug=False)
        self.conn.send(cmd_bytes)
        rospy.loginfo(f'{self.current_pos}\t{self.current_quaternion}\t{self.current_vel}\t{self.current_yawSpeed}\t{self.goal_pos}\t{vel.x}\t{vel.y}')

    def goal_pos_callback(self, msg):
        self.goal_pos = np.array([msg.x , msg.y ,msg.z])

    def pub(self):
        self.get_state()

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.current_pos[0]
        odom.pose.pose.position.y = self.current_pos[1]
        odom.pose.pose.position.z = self.current_pos[2]

        odom.pose.pose.orientation.w = self.current_quaternion[0]
        odom.pose.pose.orientation.x = self.current_quaternion[1]
        odom.pose.pose.orientation.y = self.current_quaternion[2]
        odom.pose.pose.orientation.z = self.current_quaternion[3]

        odom.twist.twist.linear.x = self.current_vel[0]
        odom.twist.twist.linear.y = self.current_vel[1]

        odom.twist.twist.angular.z = self.current_yawSpeed
        self.loc_pub.publish(odom)

def main():
    rospy.init_node('my_node')
    rospy.loginfo("go1_node_on")
    _odom_topic = rospy.get_param("/odom_topic")
    is_gazebo = rospy.get_param("/is_gazebo")

    loc_go1_pub = rospy.Publisher(_odom_topic, Odometry, queue_size=10)
    go1 = Go1(loc_go1_pub, is_gazebo)
    rospy.Subscriber('/cmd_vel', Twist, go1.cmd_vel_callback)
    rospy.Subscriber('/goal_pos_go1', Point, go1.goal_pos_callback)
    rospy.spin()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rate.sleep(0.01)

if __name__ == '__main__':
    main()