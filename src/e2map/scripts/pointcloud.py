#!/usr/bin/env python
import os

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Header


def convert_npy_to_pc(path, x_start, x_end, y_start, y_end, res, x_sample_num, y_sample_num, frame_id):
    """convert numpy array (2d) to publishable PointCloud2 message."""
    # load the numpy array with the costmap
    costmap_array = np.load(path)

    # dense x, y points
    x = np.linspace(x_start * res, x_end * res, x_sample_num)  
    y = np.linspace(y_start * res, y_end * res, y_sample_num)
    xx, yy = np.meshgrid(x, y, indexing='ij')

    # get Z values from the costmap array
    zz = costmap_array.flatten()

    # create points with x, y, z
    points = []
    for xi, yi, zi in zip(xx.flatten(), yy.flatten(), zz):
        points.append([yi, xi, zi * 2]) #scale down 

    # convert points to ROS PointCloud2 format
    header = Header()
    header.frame_id = frame_id  # set the frame of reference
    header.stamp = rospy.Time.now()

    pointcloud = point_cloud2.create_cloud_xyz32(header, points)
    return pointcloud


if __name__ == '__main__':
    # ROS node initialization
    rospy.init_node('pointcloud_viewer', anonymous=True)
    rospy.loginfo('starting costmap point cloud viewer..')
    
    frame_id = rospy.get_param("/map_frame_id")

    # publisher for the point cloud
    pub = rospy.Publisher('/cost_map_pc', PointCloud2, queue_size=10)

    # load path config
    e2map_pkg_directory = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    data_path = rospy.get_param("data_path", os.path.join(e2map_pkg_directory, 'data'))
    map_save_dir = os.path.join(data_path, "maps")

    # get cost map path
    cost_map_name = rospy.get_param("cost_map_name", "costmap.npy")
    cost_map_path = os.path.join(map_save_dir, cost_map_name)

    fps = rospy.get_param('/pointcloud_viewer/rate', 1)
    rate = rospy.Rate(fps)

    # publish
    while not rospy.is_shutdown():
        try:
            # get visualization related parameters
            x_start = rospy.get_param('/pointcloud_viewer/x_start', -123)
            x_end = rospy.get_param('/pointcloud_viewer/x_end', 123)
            y_start = rospy.get_param('/pointcloud_viewer/y_start', -123)
            y_end = rospy.get_param('/pointcloud_viewer/y_end', 123)
            res = rospy.get_param('/pointcloud_viewer/resolution', 0.05)
            x_sample_num = rospy.get_param('/pointcloud_viewer/x_sample_num', 246)
            y_sample_num = rospy.get_param('/pointcloud_viewer/y_sample_num', 356)
            
            # publish the point cloud
            pc = convert_npy_to_pc(cost_map_path, x_start, x_end, y_start, y_end, res, x_sample_num, y_sample_num, frame_id)
            pub.publish(pc)
            
            rate.sleep()
        except rospy.ROSTimeMovedBackwardsException:
            continue
        except KeyboardInterrupt:
            rospy.loginfo('shutting down pointcloud viewer')    
    
