#!/usr/bin/env python
import os
import os.path as osp
import math
import json
import sys
import re
import time

e2map_pkg_path = osp.dirname(osp.dirname(osp.abspath(__file__)))
source_path = osp.dirname(e2map_pkg_path)
sys.path.append(source_path)

import rospy
import cv2
import numpy as np
import geometry_msgs.msg as geometry_msgs
from PIL import Image
from tqdm import tqdm
from tf.transformations import quaternion_from_euler
from scipy.stats import multivariate_normal
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header, Bool
from munch import Munch
from cv_bridge import CvBridge

from foundations.utils.sftp import SFTP


class E2MapArgs():
    def __init__(self):
        # ROS
        rospy.init_node('map_node', anonymous=True)
        rospy.loginfo('get arguments for E2Map..')
        self.map_frame_id = rospy.get_param("/map_frame_id")
        self.resol = rospy.get_param("/map_resolution")
        self.rgb_sub_topic = rospy.get_param("/color_topic")
        self.cost_map_topic = rospy.get_param("/cost_map_topic")
        self.obstacle_map_topic = rospy.get_param("/obstacle_map_topic")
        self.point_cloud_topic = rospy.get_param("/point_cloud_topic")
        self.map_init_topic = rospy.get_param("/map_init_topic")
        self.event_topic = rospy.get_param("/collision_detection_topic")
        self.localization_topic = rospy.get_param("/localization_topic")
        self.update_finished_topic = rospy.get_param("/update_finished_topic")
        self.method = rospy.get_param('method')
        self.data_path = rospy.get_param(rospy.get_name() + "/data_path")
        self.obstacle_map_name = rospy.get_param(rospy.get_name() + "/obstacle_map_name")
        self.cost_map_name = rospy.get_param(rospy.get_name() + "/cost_map_name")
        self.fps = rospy.get_param(rospy.get_name() + "/fps")
        self.cost_radius = rospy.get_param(rospy.get_name() + '/cost_radius')
        self.pdf_threshold = rospy.get_param(rospy.get_name() + '/pdf_threshold')
        self.point_cloud_z_scale = rospy.get_param(rospy.get_name() + '/point_cloud_z_scale')

        # sftp
        self.hostname = rospy.get_param(rospy.get_name() + "/hostname")
        self.username = rospy.get_param(rospy.get_name() + "/username")
        self.password = rospy.get_param(rospy.get_name() + "/password")
        self.rgb_filename = rospy.get_param(rospy.get_name() + "/rgb_filename")
        self.event_filename = rospy.get_param(rospy.get_name() + "/event_filename")
        self.emotion_filename = rospy.get_param(rospy.get_name() + "/emotion_filename")
        remote_path = osp.join('/home', f'{self.username}', 'e2map')
        self.foundations_pkg_path = osp.join(source_path, 'foundations')
        self.rgb_local_dir_path = osp.join(self.foundations_pkg_path, 'event_images')
        self.rgb_remote_dir_path = osp.join(remote_path, 'images')
        self.update_local_dir_path = osp.join(self.foundations_pkg_path, 'update_output')
        self.update_remote_dir_path = osp.join(remote_path, 'texts')

        # e2map update
        self.pose_buffer_size = rospy.get_param(rospy.get_name() + "/pose_buffer_size")
        self.rgb_buffer_size = rospy.get_param(rospy.get_name() + "/rgb_buffer_size")
        self.event_timegap = rospy.get_param(rospy.get_name() + "/event_timegap")
        self.tau = rospy.get_param(rospy.get_name() + "/tau")
        self.update_rule = rospy.get_param(rospy.get_name() + "/update_rule")
        self.cov_coef = rospy.get_param(rospy.get_name() + "/cov_coef")
        self.n_neighbors = rospy.get_param(rospy.get_name() + "/n_neighbors")

        
class E2Map():
    def __init__(self, args):
        # ROS
        rospy.loginfo('starting E2Map..')
        self._map_frame_id = args.map_frame_id 
        self._resol = args.resol 
        self._rgb_sub_topic = args.rgb_sub_topic 
        self._cost_map_topic = args.cost_map_topic
        self._obstacle_map_topic = args.obstacle_map_topic 
        self._point_cloud_topic = args.point_cloud_topic
        self._map_init_topic = args.map_init_topic
        self._event_topic = args.event_topic 
        self._localization_topic = args.localization_topic
        self._update_finished_topic = args.update_finished_topic
        self._data_path = args.data_path
        self._obstacle_map_name = args.obstacle_map_name
        self._cost_map_name = args.cost_map_name
        self._cost_radius = args.cost_radius
        self._pdf_threshold = args.pdf_threshold
        self._point_cloud_z_scale = args.point_cloud_z_scale
        self._obstacle_map_msg = None
        self._cost_map_msg = None
        self._update_finished_msg = Bool(True)
        self._point_cloud_msg = None
        self._load_map_time = rospy.Time.now() 
        
        # obstacle map
        map_save_dir = osp.join(self._data_path, "maps")
        obstacle_map_path = osp.join(map_save_dir, "obstacles.npy")
        self._obstacle_map = self.load_map(obstacle_map_path)
        self._obs_height = self._obstacle_map.shape[0]
        self._obs_width = self._obstacle_map.shape[1]
        self._map_origin_vals = (-self._obs_width / 2 * self._resol, -self._obs_height / 2 * self._resol, 0)
        self._hline = np.arange(0, self._obs_height)
        self._wline = np.arange(0, self._obs_width)
        
        # sftp
        self._hostname = args.hostname
        self._username = args.username
        self._password = args.password
        self._rgb_filename = args.rgb_filename
        self._event_filename = args.event_filename
        self._emotion_filename = args.emotion_filename
        self._foundations_pkg_path = args.foundations_pkg_path
        self._rgb_local_dir_path = args.rgb_local_dir_path  
        self._rgb_remote_dir_path = args.rgb_remote_dir_path  
        self._update_local_dir_path = args.update_local_dir_path
        self._update_remote_dir_path = args.update_remote_dir_path
        
        # e2map update
        self._pose_buffer_size = args.pose_buffer_size
        self._rgb_buffer_size = args.rgb_buffer_size
        self._event_timegap = args.event_timegap
        self._cov_coef = args.cov_coef
        self._tau = args.tau
        self._update_rule = args.update_rule
        self._n_neighbors = args.n_neighbors
        self._cvbridge = CvBridge()
        self._rgb_buffer = [] 
        self._pose_buffer = [] 
        self._coord_to_emotion_param = {}
        self._idx_to_contour = {} 
        self._centroid_to_label = {}
        self._event_occured = False
        self._coord_at_event = (None, None)
        self._emotion = None
        self._prev_h_gap = None
        self._prev_w_gap = None
        self._image_at_event = None
        self._event_moment_idx = None
        self._rgb_idx = -1
        
        # ROS nodes
        self.obs_pub = rospy.Publisher(self._obstacle_map_topic, OccupancyGrid, queue_size=10)
        self.cost_pub = rospy.Publisher(self._cost_map_topic, OccupancyGrid, queue_size=10)
        self.pc_pub = rospy.Publisher(self._point_cloud_topic, PointCloud2, queue_size=10)
        self.map_init_pub = rospy.Publisher(self._map_init_topic, Bool, queue_size=10)
        self.update_finished_pub = rospy.Publisher(self._update_finished_topic, Bool, queue_size=10)
        self.pose_sub = rospy.Subscriber(self._localization_topic, Odometry, self.pose_callback)
        self.rgb_sub = rospy.Subscriber(self._rgb_sub_topic, ImageMsg, self.rgb_callback) 
        self.event_sub = rospy.Subscriber(self._event_topic, Bool, self.event_callback)
        self.rate = rospy.Rate(args.fps)
        self.method = args.method

        self.sftp = SFTP(args.hostname, args.username, args.password)
        self.sftp.connect()
        
        self._set_obstacle_map()
        self._save_all_contour_in_obstacle_map()
        self._set_initial_emotion_param()
        self._set_initial_cost_map()
    
    @property
    def coord_at_event(self):
        return self._coord_at_event
    
    @coord_at_event.setter
    def coord_at_event(self, coord):
        if coord[0] < 0 or coord[1] < 0:
            raise ValueError("coordinate must be positive values.")
        self._coord_at_event = coord
        
    @property
    def event_occured(self):
        return self._event_occured
    
    @event_occured.setter
    def event_occured(self, flag):
        if not isinstance(flag, bool):
            raise ValueError("event flag must be bool type.")
        self._event_occured = flag
    
    @staticmethod
    def get_centroid(contour):
        M = cv2.moments(contour)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return (cX, cY)
    
    @staticmethod    
    def scale_zero_to_hundred(map_data):
        data_min = np.min(map_data)
        data_max = np.max(map_data)
        scaled_map = (map_data - data_min) / (data_max - data_min) * 100 
        scaled_map = scaled_map.astype(int)
        return scaled_map

    @staticmethod    
    def load_txt(file):
        with open(file, 'r') as f:
            text = f.read()
        return text

    @staticmethod
    def load_map(load_path):
        with open(load_path, "rb") as f:
            map_data = np.load(f)
        return map_data
    
    @staticmethod
    def load_json(load_path):
        with open(load_path, "rb") as f:
            json_data = json.load(f)
        return json_data
    
    @staticmethod
    def get_euclidean_distance(node, target_nodes):
        """Compute distances from node to target_nodes, then return argmin and min distance value."""
        target_nodes = np.asarray(target_nodes)
        deltas = target_nodes - node
        return np.linalg.norm(deltas)
       
    @staticmethod
    def find_contours(image, thresh_val):
        """Find existing contours in image by tresholding given value."""
        _, thresh = cv2.threshold(image, thresh_val, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        return contours
    
    def _convert_to_pixel_coordinate(self, world_coord):
        """Convert global world's coordinate to the image coordinate."""
        x = world_coord[0]
        y = world_coord[1]
        
        pixel_x = int(x / self._resol + self._obs_height / 2)
        pixel_y = int(-y / self._resol + self._obs_width / 2)
        return (pixel_x, pixel_y)

    def _crop_map(map_data):
        row_axis, col_axis = np.where(map_data == 0)
        min_row = np.min(row_axis)  # minimum row index containing a zero
        max_row = np.max(row_axis)  # maximum row index containing a zero
        min_col = np.min(col_axis)  # minimum column index containing a zero
        max_col = np.max(col_axis)  # maximum column index containing a zero

        cropped_map = map_data[min_row : max_row+1, min_col : max_col+1]
        return cropped_map

    def _find_update_target(self, n_neighbors=0):
        """Find target occupied cell's coordinate to update."""
        raw_contours = []
        for v in self._idx_to_contour.values():
            raw_contours.append(v.raw)

        min_dist = math.inf
        update_target_coord = None
        min_contour_idx = None
        min_point_idx = None

        # traverse all points-on-contour and find the closest point
        for rc_idx, raw_contour in enumerate(raw_contours):
            for cp_idx, contour_point in enumerate(raw_contour):
                dist = self.get_euclidean_distance(self._coord_at_event, contour_point)
                if dist < min_dist:
                    min_dist = dist
                    update_target_coord = contour_point
                    min_contour_idx = rc_idx
                    min_point_idx = cp_idx
                    
        if n_neighbors == 0:
            update_target = [update_target_coord]
        else:
            update_target = raw_contours[min_contour_idx][min_point_idx-n_neighbors : min_point_idx+n_neighbors+1]
        
        return update_target

    def _compute_update_amount(self, emotion_score, tau=0.1, update_rule='wf'):
        """Compute update amount of covariance with specific update rule."""
        if update_rule == 'linear':
            return (1 + emotion_score / tau)
        elif update_rule == 'wf': # Weber-Fechner 
            return np.log(emotion_score / tau)
        else:
            raise NotImplementedError("other scaling type doesn't exist.")

    def _update_covariance(self, cov, score, target_coord):
        """Compute new variances considering unit vector."""
        dh = abs(self._coord_at_event[0] - target_coord[0])
        dw = abs(self._coord_at_event[1] - target_coord[1])
        new_cov = cov.copy()
        
        # if dh > dw: -> along height axis
        new_cov[0][0] = new_cov[0][0] * pow(score, 2) * pow(dh / np.sqrt(dh**2 + dw**2), 2)
        # elif dw > dh: -> along width axis
        new_cov[1][1] = new_cov[1][1] * pow(score, 2) * pow(dw / np.sqrt(dh**2 + dw**2), 2)
        return new_cov
    
    def _compute_multivariate_gaussian(self, x_indices, y_indices, mean, cov):
        """Compute pdf values for 2D multivariate gaussian."""
        pos = np.column_stack((x_indices, y_indices))
        rv = multivariate_normal(mean=mean, cov=cov)
        return rv.pdf(pos)
    
    def _update_e2map(self, update_target, score=3):
        """Update target cells' weight parameter and covariance."""
        # scale emotion score
        scaled_score = self._compute_update_amount(score, tau=self._tau, update_rule=self._update_rule)
        
        # cache target points 
        target_list = []
        for t in update_target:
            t = tuple(t[0])
            t = (t[1], t[0])
            target_list.append(t)
        
        # update emotion for each target point
        for target_coord in target_list:
            prev_h_i, prev_w_j = self._compute_meshgrid(target_coord, self._prev_h_gap, self._prev_w_gap)
            prev_weight = self._coord_to_emotion_param[target_coord].weight
            prev_cov = self._coord_to_emotion_param[target_coord].cov
            prev_pdfs = self._compute_multivariate_gaussian(prev_h_i.flat, prev_w_j.flat, target_coord, prev_cov)
            prev_pdfs = prev_pdfs.reshape(prev_h_i.shape)

            # update covariance 
            new_cov = self._update_covariance(prev_cov, scaled_score, target_coord)

            # compute new pdf
            new_h_gap, new_w_gap = self._compute_gap_for_threshold_pdf(new_cov, self._pdf_threshold)
            new_h_i, new_w_j = self._compute_meshgrid(target_coord, new_h_gap, new_w_gap)
            new_pdfs = self._compute_multivariate_gaussian(new_h_i.flat, new_w_j.flat, target_coord, new_cov)
            new_pdfs = new_pdfs.reshape(new_h_i.shape)
            
            # compute new weight parameter of target pixel
            prev_z = self._compute_multivariate_gaussian(target_coord[0], target_coord[1], target_coord, prev_cov)
            new_z = self._compute_multivariate_gaussian(target_coord[0], target_coord[1], target_coord, new_cov)
            new_weight = np.true_divide(prev_weight * prev_z, new_z)
            
            # update e2map
            self._emotion[prev_h_i, prev_w_j] -= prev_weight * prev_pdfs
            self._emotion[new_h_i, new_w_j] += new_weight * new_pdfs
            
            # update emotion parameters
            self._coord_to_emotion_param[target_coord].update(cov=new_cov)
            self._coord_to_emotion_param[target_coord].update(weight=new_weight)

        # publish updated e2map & pointcloud
        self._infill_cost_map_data(self._emotion)
        self._set_cost_map_point_cloud(self._emotion)
        
        # set new gaps
        self._prev_h_gap = new_h_gap
        self._prev_w_gap = new_w_gap
        
    def _compute_gap_for_threshold_pdf(self, cov, pdf_threshold):
        """Compute valid pdf region, w.r.t pdf_threshold value."""
        h_sigma = math.sqrt(cov[0][0])
        w_sigma = math.sqrt(cov[1][1])
        log_val = np.log(1 / (2 * np.pi * h_sigma * w_sigma * pdf_threshold + 1e-10))
        if log_val > 0:
            h_gap = h_sigma * math.sqrt(2 * log_val)
            w_gap = w_sigma * math.sqrt(2 * log_val)
            return int(np.round(h_gap)), int(np.round(w_gap))
        else:
            raise ValueError("Can't compute gap, since log value is negative.")

    def _set_occupancy_grid(self):
        """Return map_instance of OccuapncyGrid messasge w.r.t obstacle map's meta data."""
        msg = OccupancyGrid()
        msg.header.frame_id = self._map_frame_id
        msg.info.map_load_time = rospy.Time.now()
        msg.info.width = self._obs_width
        msg.info.height = self._obs_height
        msg.info.resolution = self._resol
        origin = geometry_msgs.Pose()
        origin.position = geometry_msgs.Point(x=self._map_origin_vals[0], y=self._map_origin_vals[1], z=0)
        q = quaternion_from_euler(0, 0, self._map_origin_vals[2])
        origin.orientation = geometry_msgs.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        msg.info.origin = origin
        return msg
    
    def _set_obstacle_map(self):
        """Wraps numpy obstacle map to OccupancyGrid message."""
        self._obstacle_map_msg = self._set_occupancy_grid()

        # stack from last row of 2D array map data to consider occpancyGrid data stacking order
        for i in reversed(range(self._obs_height)):
            for j in range(self._obs_width):
                if self._obstacle_map[i, j] == 1:
                    self._obstacle_map_msg.data.append(0)
                elif self._obstacle_map[i, j] == 0:
                    self._obstacle_map_msg.data.append(100)
                else:
                    raise ValueError("Your obstacle.npy includes abnormal value. Check your obstacle.npy whether contains binary value (0/1).")
                
        half_margin = math.floor(self._cost_radius / 2)
        for i in range(self._obs_height):
            for j in range(self._obs_width):
                if self._obstacle_map_msg.data[i * self._obs_width + j] == 100:
                    for k in range(self._cost_radius):
                        for l in range(self._cost_radius):
                            if self._obstacle_map_msg.data[(i - half_margin + k)*self._obs_width + j - half_margin + l] != 100:
                                max_val = np.max([self._obstacle_map_msg.data[(i - half_margin + k)*self._obs_width + j - half_margin + l], 100 - 20 * np.sqrt(pow(k - half_margin, 2.0) + pow(l - half_margin, 2.0))])
                                self._obstacle_map_msg.data[(i - half_margin + k)*self._obs_width + j - half_margin + l] = int(max_val)
 
    def _save_all_contour_in_obstacle_map(self):
        """Save dictionary that contour index (Key) : Munch('raw': raw contour (2D array), 'size': num of pixels in the contour) (Value)."""
        reversed_obs = 1 - self._obstacle_map # make 1: obstacle & 0: free
        contours = self.find_contours(reversed_obs, 0.5)
        for idx, contour in enumerate(contours):
            data = Munch()
            if idx not in self._idx_to_contour:
                num_pixels = self._count_pixel_num_of_contour(contour)
                data.raw = contour
                data.size = num_pixels
                self._idx_to_contour[idx] = data 

    def _count_pixel_num_of_contour(self, contour):
        """Count the number of pixels lay inside the contour."""
        background = np.zeros_like(self._obstacle_map, dtype=np.uint8)
        drawed = cv2.drawContours(background, contour, -1, (255,0,0), 1, cv2.LINE_AA)
        return np.count_nonzero(drawed)
    
    def _compute_meshgrid(self, coord, h_gap, w_gap):
        """Return surrounding meshgrid w.r.t given coordinate, considering gaps."""
        h_min = np.maximum(0, coord[0]-h_gap)
        h_max = np.minimum(coord[0]+h_gap, self._obs_height-1)
        w_min = np.maximum(0, coord[1]-w_gap)
        w_max = np.minimum(coord[1]+w_gap, self._obs_width-1)
        
        new_hline = self._hline[h_min : h_max]
        new_wline = self._wline[w_min : w_max]
        h_i, w_j = np.meshgrid(new_hline, new_wline, indexing='ij') # array coordinate
        return h_i, w_j

    def _set_initial_emotion_param(self):
        """Set initial parameters of emotion 1) covariance 2) initial weighting factor for occupied grid cells."""
        zero_height_indices, zero_width_indices = np.where(self._obstacle_map == 0)

        gaussian_counter = np.zeros_like(self._obstacle_map, dtype=float)
        
        for i, j in zip(zero_height_indices, zero_width_indices):
            # cache Gaussian
            mode = Munch()
            mode.cov = np.identity(2) * self._cov_coef
            mode.weight = None
            
            # count number of surrounding neighbors
            h_gap, w_gap = self._compute_gap_for_threshold_pdf(mode.cov, self._pdf_threshold)
            h_i, w_j = self._compute_meshgrid([i, j], h_gap, w_gap)
            gaussian_counter[h_i, w_j] += 1
            
            self._coord_to_emotion_param[tuple([i, j])] = mode
                
        for i, j in zip(zero_height_indices, zero_width_indices):
            self._coord_to_emotion_param[tuple([i, j])].weight = 1  / gaussian_counter[i, j]
    
    def _set_initial_cost_map(self):
        """Dynamically make initial cost_map from obstacle map, 
        assigning Gaussian with unit height and identity covariance on occupied cell in in obstacle map.
        """
        # set initial emotion parameters for occupied grid pixel
        self._cost_map_msg = self._set_occupancy_grid()
        self.updated_cost_map_msg = self._set_occupancy_grid()
        
        zero_height_indices, zero_width_indices = np.where(self._obstacle_map == 0)
        
        # compute initial thresholding gap
        cov = np.identity(2) * self._cov_coef
        h_gap, w_gap = self._compute_gap_for_threshold_pdf(cov, self._pdf_threshold)
        self._prev_h_gap = h_gap
        self._prev_w_gap = w_gap
        
        self._emotion = np.zeros_like(self._obstacle_map, dtype=float)

        # assign unit bivariate gaussian for occupied cells
        for i, j in zip(tqdm(zero_height_indices, desc="set initial cost_map"), zero_width_indices):
            mu = [i, j]
            
            # get surrounding meshgrid
            h_i, w_j = self._compute_meshgrid(mu, h_gap, w_gap)

            # thresholded pdf
            rv = multivariate_normal(mean=mu, cov=self._coord_to_emotion_param[tuple([i, j])].cov)
            cropped = np.column_stack((h_i.flat, w_j.flat))
            cost_pdf = rv.pdf(cropped) 
            cost_pdf = cost_pdf.reshape(h_i.shape)
            
            # assign pdf for surrounding region
            self._emotion[h_i, w_j] += self._coord_to_emotion_param[tuple([i, j])].weight * cost_pdf
            
        self._infill_cost_map_data(self._emotion)
        self._set_cost_map_point_cloud(self._emotion)
        self.map_init_pub.publish(Bool(True))
        
    def _set_cost_map_point_cloud(self, map_data):
        """Allocate cost map corresponding pointcloud message."""
        map_data = self.scale_zero_to_hundred(map_data)
        hline = np.linspace(-self._obs_height/2 * self._resol, self._obs_height/2 * self._resol, self._obs_height)
        wline = np.linspace(-self._obs_width/2 * self._resol, self._obs_width/2 * self._resol, self._obs_width)
        h_i, w_j = np.meshgrid(hline, wline, indexing='ij') # array coordinate

        points = []
        for i, j, z in zip(h_i.flatten(), w_j.flatten(), map_data.flatten()):
            if z != 0:
                points.append([j, -i, z / self._point_cloud_z_scale])
        
        header = Header()
        header.frame_id = self._map_frame_id
        self._point_cloud_msg = point_cloud2.create_cloud_xyz32(header, points)            
    
    def _infill_cost_map_data(self, map_data):
        """Infill message data of cost map."""
        map_data = self.scale_zero_to_hundred(map_data)
        self._cost_map_msg.data = []
        for i in reversed(range(self._obs_height)):
            for j in range(self._obs_width):
                self._cost_map_msg.data.append(map_data[i, j])

    def make_dirs(self):
        """Make local directories for saving egocentric RGBs & E2Map update related outputs."""
        if not osp.exists(self._rgb_local_dir_path):
            print('make dir for egocentric rgb..')
            os.makedirs(self._rgb_local_dir_path)
        if not osp.exists(self._update_local_dir_path):
            print('make dir for update output..')
            os.makedirs(self._update_local_dir_path) 
                    
    def clear_history(self):
        """Clear all previous event description & update evaluation result."""
        rospy.loginfo('clear previous outputs in the remote..')
        if len(self.sftp.connection.listdir(self._rgb_remote_dir_path)) != 0:
            for item in self.sftp.connection.listdir(self._rgb_remote_dir_path):
                self.sftp.connection.remove(osp.join(self._rgb_remote_dir_path, item))
        if len(self.sftp.connection.listdir(self._update_remote_dir_path)) != 0:
            for item in self.sftp.connection.listdir(self._update_remote_dir_path):
                self.sftp.connection.remove(osp.join(self._update_remote_dir_path, item))
    
    def upload_event_images(self):
        """Transmit three egocentric RGB images saved in local to the remote."""
        # wait until rgb stacks more than timegap (h) images from event moment
        while self._rgb_idx < self._event_moment_idx + self._event_timegap:
            pass
        
        # t_event - h, t_event, t_event + h
        event_imgs = self._rgb_buffer[::-1][::self._event_timegap][:3]
        
        for i, img_npy in enumerate(reversed(event_imgs)):
            img = Image.fromarray(img_npy)
            local_img_path = osp.join(self._rgb_local_dir_path, self._rgb_filename + f'_{i}.jpg')
            remote_img_path = osp.join(self._rgb_remote_dir_path, self._rgb_filename + f'_{i}.jpg')
            img.save(local_img_path)
            self.sftp.connection.put(local_img_path, remote_img_path)
        
        print("Upload event images!")
        
    def get_emotion_score(self):
        """Download update outputs from the remote to the local, then compute emotion scores"""
        event_file = f'{self._event_filename}.txt' # download event description result for explainability
        emotion_file = f'{self._emotion_filename}.txt'
        
        remote_event_path = osp.join(self._update_remote_dir_path, event_file)
        remote_emotion_path = osp.join(self._update_remote_dir_path, emotion_file)
        local_event_path = osp.join(self._update_local_dir_path, event_file)
        local_emotion_path = osp.join(self._update_local_dir_path, emotion_file)

        while not self.sftp.connection.isfile(remote_emotion_path):
            print('waiting for emotion evaluation result..')
            time.sleep(1)

        if self.sftp.connection.isfile(remote_emotion_path):
            self.sftp.download(remote_event_path, local_event_path)
            self.sftp.download(remote_emotion_path, local_emotion_path)
            output = self.load_txt(local_emotion_path)
            scores = re.findall(r'\d+', output) # extract only numbers
            emotion_score = int(scores[2])
            return emotion_score
        else:
            raise ValueError(f"{remote_emotion_path} is not a file!")
    
    def update(self):
        """E2Map's emotion update pipeline."""
        # send event images
        self.upload_event_images()

        # get emotion score result
        emotion_score = self.get_emotion_score()

        # find target point to update
        target_coord = self._find_update_target(self._n_neighbors)

        # update target point's gaussian mode
        self._update_e2map(target_coord, score=emotion_score)

        # publish flag for update is done
        self.update_finished_pub.publish(self._update_finished_msg)
    
    def publish_obstacle_map(self):
        assert self._obstacle_map_msg, "obstacle map should be defined"
        self._obstacle_map_msg.header.stamp = self._load_map_time
        self.obs_pub.publish(self._obstacle_map_msg)

    def publish_cost_map(self):
        assert self._cost_map_msg, "cost map should be defined"
        self._cost_map_msg.header.stamp = self._load_map_time
        self.cost_pub.publish(self._cost_map_msg)
        
    def publish_point_cloud(self):
        assert self._point_cloud_msg, "point cloud message should be filled"
        self._point_cloud_msg.header.stamp = self._load_map_time
        self.pc_pub.publish(self._point_cloud_msg)
    
    def event_callback(self, msg):
        occured = msg.data
        if occured and self.event_occured == False:
            self.event_occured = occured
            self._event_moment_idx = self._rgb_idx
            last_pose = self._pose_buffer[-1]
            self.coord_at_event = (last_pose[0], last_pose[1]) 
            
    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        x, y = self._convert_to_pixel_coordinate((pos.x, pos.y))
        pose = np.array([x, y, pos.z, quat.x, quat.y, quat.z, quat.w]) 
        self._pose_buffer.append(pose)
        if len(self._pose_buffer) > self._pose_buffer_size:
            self._pose_buffer.pop(0)
    
    def rgb_callback(self, msg):
        rgb_img = self._cvbridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        rgb_npy = np.array(rgb_img)
        self._rgb_buffer.append(rgb_npy)
        self._rgb_idx += 1
        if len(self._rgb_buffer) > self._rgb_buffer_size:
            self._rgb_buffer.pop(0)
    
    
if __name__ == '__main__':
    args = E2MapArgs()
    e2map = E2Map(args)
    e2map.make_dirs()
    e2map.clear_history()
    
    while not rospy.is_shutdown():
        try:
            e2map.publish_cost_map()
            e2map.publish_obstacle_map()
            e2map.publish_point_cloud()
            e2map.rate.sleep()
            rospy.loginfo("Publishing maps..")
            
            if e2map.event_occured:
                if e2map.method == 'e2map':
                    e2map.update()
                    e2map.clear_history()
                elif e2map.method in ['vlmap', 'lm_nav']:
                    e2map.update_finished_pub.publish(e2map._update_finished_msg)
                e2map.event_occured = False

        except rospy.ROSTimeMovedBackwardsException:
            continue
        except KeyboardInterrupt:
            rospy.loginfo('shutting down E2Map')    
    
