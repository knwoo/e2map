#!/usr/bin/env python
import sys
import os
from collections import Counter

import numpy as np
import math
import matplotlib.pyplot as plt
import cv2
import torch
import clip
import matplotlib.patches as plp
from PIL import Image

import matplotlib.patches as mpatches

from subgoal_filter_parser import parse_configs
upper_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(upper_dir)
from utils.mp3dcat import mp3dcat


class SubgoalCoordinateFilter():
    def __init__(self, args):
        self.data_path = args.data_path
        self.mask_version = args.mask_version
        self.clip_version = args.clip_version
        self.min_area = args.min_area
        self.max_area = args.max_area
        self.margin = args.margin
        self.do_crop = args.do_crop
        self.device = args.device
        self.goal_arrived_thresh = args.goal_arrived_thresh/args.map_resolution
        self.raycast_resolution = args.raycast_resolution
        
        self.map_save_dir = os.path.join(args.data_path, "maps")
        self.grid_save_path = os.path.join(self.map_save_dir, f"grid_lseg_{args.mask_version}.npy")
        self.obstacles_save_path = os.path.join(self.map_save_dir, "obstacles.npy")
        
        self.clip_model = None
        self.clip_feat_dim = None
        
        self.obstacles = None
        
        self.grid = None
        self.seg = None
        self.xmin = None
        self.xmax = None
        self.ymin = None
        self.ymax = None
        
        self.subgoal_dict = {}
        
        self.contour_draw_list = []
        self.patches = None

        self.prepare_maps_and_map_params()
        self.load_clip_model()
        
        self.robot_pixel_pose = [self.obstacles.shape[1] / 2, self.obstacles.shape[0] / 2]
    
    @staticmethod
    def load_map(load_path):
        with open(load_path, "rb") as f:
            map_data = np.load(f)
        return map_data
    
    def find_point(self, node, target_nodes, direction=None):
        """Find nearest point or directed point."""
        if direction == 'left':
            return self.find_far_left_point(target_nodes)
        elif direction == 'right':
            return self.find_far_right_point(target_nodes)
        elif direction == 'top':
            return self.find_far_top_point(target_nodes)
        elif direction == 'bottom':
            return self.find_far_bottom_point(target_nodes)
        else:
            return self.find_closest_point(node, target_nodes)
    
    @staticmethod
    def get_euclidean_distance(node, target_nodes):
        """Compute distances from node to target_nodes, then return argmin and min distance value."""
        target_nodes = np.asarray(target_nodes)
        deltas = target_nodes - node
        return np.linalg.norm(deltas)
    
    @staticmethod
    def find_closest_point(node, target_nodes):
        """Compute distances from node to target_nodes, then return argmin and min distance value."""
        target_nodes = np.asarray(target_nodes)
        deltas = target_nodes - node
        dist_2 = np.einsum('ij,ij->i', deltas, deltas)
        return np.argmin(dist_2)
    
    @staticmethod
    def find_far_left_point(target_nodes):
        """Return far left node's index."""
        target_nodes = np.asarray(target_nodes)
        
        return np.argmin(target_nodes[:, 0])
    
    @staticmethod
    def find_far_right_point(target_nodes):
        """Return far right node's index."""
        target_nodes = np.asarray(target_nodes)
        return np.argmax(target_nodes[:, 0])
    
    @staticmethod
    def find_far_top_point(target_nodes):
        """Return far top node's index."""
        target_nodes = np.asarray(target_nodes)
        return np.argmin(target_nodes[:, 1])
    
    @staticmethod
    def find_far_bottom_point(target_nodes):
        """Return far bottom node's index."""
        target_nodes = np.asarray(target_nodes)
        return np.argmax(target_nodes[:, 1])
    
    @staticmethod    
    def get_internal_division_point(start, goal, margin=10.0):
        """Compute internal division point with given margin (fixed value) in the line that penetrates start to goal.
        Here, we assume margin comes out from the goal.
        """ 
        if isinstance(start, (list, tuple)) and isinstance(goal, (list, tuple)): 
            dx = goal[0] - start[0]
            dy = goal[1] - start[1]
            dist = np.sqrt(dx ** 2 + dy ** 2)
            
            # if distance is longer than a margin, return internal division point
            if dist > margin:
                residual = int(np.round(dist - margin))
                x = (margin * start[0] + residual * goal[0]) / (margin + residual)
                y = (margin * start[1] + residual * goal[1]) / (margin + residual)
                return [int(np.round(x)), int(np.round(y))]
            # if not, just return star coordinate itself
            elif dist <= margin:
                return start
        else:
            raise TypeError("start and goal both should be list or tuple type")
    
    @staticmethod
    def get_external_division_point(start, goal, margin=10.0):
        """Compute external division point of line 'start - goal' with given margin (fixed value) in the line that penetrates start to goal.
        Here, we assume the point exists near the goal.
        """ 
        if isinstance(start, (list, tuple)) and isinstance(goal, (list, tuple)): 
            dx = goal[0] - start[0]
            dy = goal[1] - start[1]
            dist = np.sqrt(dx ** 2 + dy ** 2)
            
            x = (margin * start[0] - (dist + margin) * goal[0]) / dist
            y = (margin * start[1] - (dist + margin) * goal[1]) / dist
            return [int(x), int(y)]
        else:
            raise TypeError("start and goal both should be list or tuple type")
    
    @staticmethod
    def get_circle_coordinates(center, radius):
        """Generate the coordinates of points within a circle given the center and radius."""
        x_min = center[0] - int(radius)
        x_max = center[0] + int(radius)
        y_min = center[1] - int(radius)
        y_max = center[1] + int(radius)
        
        # create a grid of x and y values within this range
        x = np.arange(x_min, x_max + 1)
        y = np.arange(y_min, y_max + 1)
        xv, yv = np.meshgrid(x, y, indexing='ij')
        
        dist_sq = (xv - center[0]) ** 2 + (yv - center[1]) ** 2
        
        # determine which points are within the specified radius
        inside = dist_sq <= radius**2
        
        return list(zip(xv[inside], yv[inside]))
    
    @staticmethod
    def deg_to_rad(degree):
        return degree * math.pi / 180
    
    @staticmethod
    def erase_element_by_index(array, index):
        array_list = list(array)
        array_list.pop(index)
        return np.array(array_list)
    
    def get_direction_angles_in_deg(self, direction, resolution):
        """Return pre-defined angles considering direction and angle resolution."""
        if direction == 'left':
            return np.arange(135, 225 + resolution, resolution)
        elif direction == 'right':
            return np.arange(-45, 45 + resolution, resolution)
        elif direction == 'top':
            return np.arange(-135, -45 + resolution, resolution)
        elif direction == 'bottom':
            return np.arange(45, 135 + resolution, resolution)
        else:
            return np.arange(0, 360 + resolution, resolution)
    
    def prepare_maps_and_map_params(self):
        """Assign map variables and related parameters."""
        self.obstacles = self.load_map(self.obstacles_save_path)
        self.grid = self.load_map(self.grid_save_path)
        
        if self.do_crop:
            x_indices, y_indices = np.where(self.obstacles == 0)
            self.xmin = np.min(x_indices)
            self.xmax = np.max(x_indices)
            self.ymin = np.min(y_indices)
            self.ymax = np.max(y_indices)
            
            self.grid = self.grid[self.xmin : self.xmax + 1, self.ymin : self.ymax + 1]
            self.obstacles = self.obstacles[self.xmin : self.xmax + 1, self.ymin : self.ymax + 1] > 0
        else:
            self.obstacles = self.obstacles > 0
        
    def load_clip_model(self):
        """Load CLIP related variables."""
        self.clip_feat_dim = {'RN50': 1024, 'RN101': 512, 'RN50x4': 640, 'RN50x16': 768,
                            'RN50x64': 1024, 'ViT-B/32': 512, 'ViT-B/16': 512, 'ViT-L/14': 768}[self.clip_version]
            
        self.clip_model, _ = clip.load(self.clip_version) 
        self.clip_model = self.clip_model.to(self.device).eval()
            
    def get_text_feats(self, in_text, clip_model, clip_feat_dim, batch_size=64):
        """Embed given text to text encoder of CLIP."""
        if torch.cuda.is_available():
            text_tokens = clip.tokenize(in_text).cuda()
        else :
            text_tokens = clip.tokenize(in_text)
        text_id = 0
        text_feats = np.zeros((len(in_text), clip_feat_dim), dtype=np.float32)
        while text_id < len(text_tokens):  # Batched infference.
            batch_size = min(len(in_text) - text_id, batch_size)
            text_batch = text_tokens[text_id : text_id + batch_size]
            with torch.no_grad():
                batch_feats = clip_model.encode_text(text_batch).float()
            batch_feats /= batch_feats.norm(dim=-1, keepdim=True)
            batch_feats = np.float32(batch_feats.cpu())
            text_feats[text_id : text_id + batch_size, :] = batch_feats
            text_id += batch_size
        return text_feats    
    
    def get_new_pallete(self, num_cls):
        """Get pallete for visualizing landmarks."""
        n = num_cls
        pallete = [0] * (n * 3)

        for j in range(0, n):
            lab = j
            pallete[j * 3 + 0] = 0
            pallete[j * 3 + 1] = 0
            pallete[j * 3 + 2] = 0
            i = 0
            while lab > 0:
                pallete[j * 3 + 0] |= ((lab >> 0) & 1) << (7 - i)
                pallete[j * 3 + 1] |= ((lab >> 1) & 1) << (7 - i)
                pallete[j * 3 + 2] |= ((lab >> 2) & 1) << (7 - i)
                i = i + 1
                lab >>= 3
        return pallete

    def get_new_mask_pallete(self, npimg, new_palette, out_label_flag=False, labels=None, ignore_ids_list=[]):
        """Get image color pallete for visualizing masks."""
        # put colormap
        out_img = Image.fromarray(npimg.squeeze().astype("uint8"))
        out_img.putpalette(new_palette)

        if out_label_flag:
            assert labels is not None
            u_index = np.unique(npimg)
            patches = []
            for i, index in enumerate(u_index):
                if index in ignore_ids_list:
                    continue
                label = labels[index]
                cur_color = [
                    new_palette[index * 3] / 255.0,
                    new_palette[index * 3 + 1] / 255.0,
                    new_palette[index * 3 + 2] / 255.0,
                ]
                red_patch = mpatches.Patch(color=cur_color, label=label)
                patches.append(red_patch)
        return out_img, patches

    def get_points_of_line_segment(self, coord_1, coord_2, num_steps=100):
        """Generate points along the line from (x0, y0) to (x1, y1) using Bresenham's algorithm."""
        x0, y0 = coord_1
        x1, y1 = coord_2
        
        dx = x1 - x0
        dy = y1 - y0
        magnitude = np.sqrt(dx ** 2 + dy ** 2)
        unit_vector = (dx / magnitude, dy / magnitude)
        
        return [(int(np.round(x0 + unit_vector[0] * step)), int(np.round(y0 + unit_vector[1] * step))) for step in range(num_steps)]
   
    def shoot_until_collision(self, start_point, angle):
        """Auxiliary method to implement bandcast algorithm."""
        x0, y0 = start_point
        unit_vector = (np.cos(angle), np.sin(angle))
        
        step = 1

        # find the farthest collision point when casting ray
        while True:
            # generate points starting from coord_2 (i.e., contour point)
            new_x = int(np.round(x0 + unit_vector[0] * step))
            new_y = int(np.round(y0 + unit_vector[1] * step))

            # if outside map, break the shooting loop
            if new_x >= self.obstacles.shape[1] or new_x < 0 \
                or new_y >= self.obstacles.shape[0] or new_y < 0:
                break

            # if faces obstacle, break the shooting loop
            if self.obstacles[new_y][new_x] == 0:
                break
            
            step += 1
            
        return np.sqrt((new_x - x0) ** 2 + (new_y - y0) ** 2), (x0, y0), unit_vector
    
    def shoot_until_no_collision(self, start_point, angle):
        """Auxiliary method to implement bandcast algorithm."""
        x0, y0 = start_point
        unit_vector = (np.cos(angle), np.sin(angle))
        
        step = 1

        # find the closest collision free point when casting ray
        while True:
            # generate points starting from coord_2 (i.e., contour point)
            new_x = int(np.round(x0 + unit_vector[0] * step))
            new_y = int(np.round(y0 + unit_vector[1] * step))

            # if outside map, break the shooting loop
            if new_x >= self.obstacles.shape[1] or new_x < 0 \
                or new_y >= self.obstacles.shape[0] or new_y < 0:
                break

            # if faces collision free grid, break the shooting loop
            if self.obstacles[new_y][new_x] != 0:
                break
            
            step += 1
            
        return np.sqrt((new_x - x0) ** 2 + (new_y - y0) ** 2), (new_x, new_y), unit_vector

    def check_obstacles(self, *args):
        if len(args) == 1 and isinstance(args[0], (list, np.array)):
            for x, y in list(args[0]):
                if self.obstacles[y][x] == 0: # assuming obstacle = 0 
                    return True
        elif len(args) == 2 and isinstance(args[0], (list, tuple)) and isinstance(args[1], (list, tuple)):
            for x, y in self.get_points_of_line_segment(args[0], args[1]):
                if self.obstacles[y][x] == 0: 
                    return True
        elif len(args) >= 3:
            raise TypeError("argument should be 1) list of coordinates or 2) two coordinate")
        return False
       
    def find_navigable_direction_with_raycast(self, contour_point, resolution=15, direction=None, reverse=False):
        """Find navigable direction of subgoal's contour via radiating rays (lines) that is originated from given contour point."""
        nav_point = None
        dir_vec_of_nav_point = None
        angles = self.get_direction_angles_in_deg(direction, resolution)
        contour_point = (contour_point[0], contour_point[1]) # np.array -> tuple
            
        if not reverse:
            max_coll_dist = -float('inf')
            
            # find the most longest ray, and save 1) boundary coordinate, 2) the projected coordinate on the obstacle, 3) distance from 1) to 2)
            for angle in angles:
                # collect ray that does not include any obstacles in the strip
                coll_dist, start_point, direct_vec = self.shoot_until_collision(contour_point, self.deg_to_rad(angle))
                if coll_dist > max_coll_dist:
                    max_coll_dist = coll_dist
                    nav_point = start_point
                    dir_vec_of_nav_point = direct_vec

        else:
            min_free_dist = float('inf')
            
            # find the most longest ray, and save 1) boundary coordinate, 2) the projected coordinate on the obstacle, 3) distance from 1) to 2)
            for angle in angles:
                # collect ray that does not include any obstacles in the strip
                free_dist, start_point, direct_vec = self.shoot_until_no_collision(contour_point, self.deg_to_rad(angle))
                if free_dist < min_free_dist:
                    min_free_dist = free_dist
                    nav_point = start_point
                    dir_vec_of_nav_point = direct_vec

        return nav_point, dir_vec_of_nav_point
    
    def find_navigable_point_with_raycast(self, contour_point, resolution=15, direction=None, reverse=False):
        """Find navigable point of subgoal's contour via radiating rays (lines) that is originated from given contour point."""
        direct_point, direct_vec = self.find_navigable_direction_with_raycast(contour_point, resolution, direction, reverse)
                        
        final_x = int(np.round((direct_point[0] + direct_vec[0] * self.margin)))
        final_y = int(np.round((direct_point[1] + direct_vec[1] * self.margin)))
        navigable_point = [final_x, final_y]
        return navigable_point
    
    def find_collision_free_point(self, contour, prev_coord, direction):
        """Find collision free point of subgoal's contour"""
        collision = True
        contour_copy = contour.copy()
        while(len(contour_copy)!=0):
            point_idx = self.find_point(prev_coord, contour_copy, direction)
            navigable_point = self.find_navigable_point_with_raycast(contour_copy[point_idx], self.raycast_resolution, direction)
            
            if self.check_obstacles(self.get_circle_coordinates(navigable_point, radius=self.margin // 2)):
                contour_copy = self.erase_element_by_index(contour_copy, point_idx)
            else:
                collision = False
                break
        
        if collision:
            contour_copy = contour.copy()
            while(len(contour_copy)!=0):
                point_idx = self.find_point(prev_coord, contour_copy, direction)
                navigable_point = self.find_navigable_point_with_raycast(contour_copy[point_idx], self.raycast_resolution, direction, reverse=True)
                if self.check_obstacles(self.get_circle_coordinates(navigable_point, radius=self.margin // 2)):
                    contour_copy = self.erase_element_by_index(contour_copy, point_idx)
                else:
                    collision = False
                    break
                
        return navigable_point, collision
    
    def find_min_distance_between_two_contour(self, contour1, contour2):
        """Find minimum distance between two contour by calculating distance between all two point of each contour."""
        min_dist = math.inf
        min_dist_point1_idx = None
        min_dist_point2_idx = None
        for point1_idx, point1 in enumerate(contour1):
            for point2_idx, point2 in enumerate(contour2):
                dist = self.get_euclidean_distance(point1, point2)
                if dist < min_dist:
                    min_dist = dist
                    min_dist_point1_idx = point1_idx
                    min_dist_point2_idx = point2_idx
        return min_dist, min_dist_point1_idx, min_dist_point2_idx
        
    def find_point_between_two_landmarks(self, contour1, contour2):
        """Find point between two landmarks' contour."""
        collision_free = False
        
        while(len(contour1) != 0 and len(contour2) != 0):
            _, point1_idx, point2_idx = self.find_min_distance_between_two_contour(contour1, contour2)
            navigable_point = ((contour1[point1_idx] + contour2[point2_idx])/2).astype(int)
            if self.check_obstacles(self.get_circle_coordinates(navigable_point, radius=self.margin // 2)):
                contour1 = self.erase_element_by_index(contour1, point1_idx)
                contour2 = self.erase_element_by_index(contour2, point2_idx)
            else:
                collision_free = True
                break
        
        return navigable_point, collision_free
                
    def set_robot_pose(self, robot_pose):
        self.robot_pixel_pose = robot_pose
    
    def set_seg(self, lang, pixel_label):
        """Set self.seg using lang and pixel label."""
        new_pallete = self.get_new_pallete(len(lang))
        mask, self.patches = self.get_new_mask_pallete(pixel_label, new_pallete, out_label_flag=True, labels=lang)
        self.seg = mask.convert("RGBA")
        self.seg = np.array(self.seg)
        self.seg[self.obstacles] = [225, 225, 225, 255]
        
    def get_subgoal_seg(self, subgoal_idx, pixel_label):
        """Get segmentation map where given subgoal is only activated."""
        subgoal_mask = pixel_label == subgoal_idx

        # remain only subgoal - labeld pixel clusters to find all contours of the target subgoals
        seg_cp = self.seg.copy()
        seg_cp[~subgoal_mask] = [225, 225, 225, 255]
        return seg_cp
        
    def get_pixel_label_and_confidence(self, lang):
        """Calculate each pixel's label and corresponding confidence."""
        # inject null query 'others' to prevent un-segmentation result for a single query case
        if 'others' not in lang:
            lang.insert(0, 'others')
        
        # embed features and compute cosine similarities
        text_feats = self.get_text_feats(lang, self.clip_model, self.clip_feat_dim)
        map_feats = self.grid.reshape((-1, self.grid.shape[-1]))
        scores_list = map_feats @ text_feats.T
        
        # make scores_list's class-wise sum be 1
        scores_list_sum = np.sum(scores_list, axis=1, keepdims=True)
        scores_list = np.divide(scores_list, scores_list_sum, np.zeros_like(scores_list), where=scores_list_sum!=0)
        
        pixel_label = np.argmax(scores_list, axis=1)
        pixel_confidence = np.max(scores_list, axis=1)
        pixel_label = pixel_label.reshape(self.obstacles.shape)
        pixel_confidence = pixel_confidence.reshape(self.obstacles.shape)
        
        return pixel_label, pixel_confidence
    
    def get_contour_from_seg(self, seg):
        """Extract contours of the object in the seg map."""
        seg_gray = cv2.cvtColor(seg, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(seg_gray, 150, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE,
                                                    cv2.CHAIN_APPROX_NONE) #CHAIN_APPROX_NONE: 모든 좌표 반환
        return contours
    
    def relabel_contours(self, contours, pixel_label, pixel_confidence):
        """Relabel contours that contain several different labeled pixel with label with maximum confidence."""
        for contour_idx, contour in enumerate(contours):
            x, y, w, h = cv2.boundingRect(contour)

            # ignore the largest contour (vlmap frame)
            if w == self.seg.shape[1] and h == self.seg.shape[0]:
                continue
            
            label_dict = {} # label(idx) : weighted (weighting factor: min-max normalized confidence) counting
            label_counter_dict = {}
            for i in range(y, y + h):
                for j in range(x, x + w):
                    # points that exists inside & on the contour
                    if cv2.pointPolygonTest(contour, (j, i), False) >= 0:
                        label = pixel_label[i, j]
                        conf = pixel_confidence[i, j]
                        
                        if label in label_dict:
                            label_dict[label] += conf
                            label_counter_dict[label] += 1
                        else:
                            label_dict[label] = conf
                            label_counter_dict[label] = 1
                        
                                    
            # find the label which has the most largest weighted counting 
            label_counts = Counter(label_dict)
            dominant_label = max(label_counts, key=label_counts.get)
            
            # continue if contour is composed of single label / or null list
            if len(label_dict.keys()) == 0 or len(label_dict.keys()) == 1:
                continue

            # relabelling pixels to the most dominant label
            for i in range(y, y + h):
                for j in range(x, x + w):
                    if cv2.pointPolygonTest(contour, (j, i), False) >= 0:
                        pixel_label[i, j] = dominant_label
        return pixel_label
    
    def get_contours_conf(self, contours, pixel_label, pixel_confidence):
        """Calculate contours confidence."""
        contour_avg_conf = {}
        contour_label = {}
        contour_conf = {}
        contour_label_count = {}
        
        for contour_idx, contour in enumerate(contours):
            x, y, w, h = cv2.boundingRect(contour)

            # ignore the largest contour (vlmap frame)
            if w == self.seg.shape[1] and h == self.seg.shape[0]:
                continue
            
            label_dict = {} # label(idx) : weighted (weighting factor: min-max normalized confidence) counting
            label_counter_dict = {}
            for i in range(y, y + h):
                for j in range(x, x + w):
                    # points that exists inside & on the contour
                    if cv2.pointPolygonTest(contour, (j, i), False) >= 0:
                        label = pixel_label[i, j]
                        conf = pixel_confidence[i, j]
                        
                        if label in label_dict:
                            label_dict[label] += conf
                            label_counter_dict[label] += 1
                        else:
                            label_dict[label] = conf
                            label_counter_dict[label] = 1
                            
            # find the label which has the most largest weighted counting 
            label_counts = Counter(label_dict)
            dominant_label = max(label_counts, key=label_counts.get)
            
            # save contour's average confidence score of the dominant label
            dominant_label_count = label_counter_dict[dominant_label]
            dominant_label_conf = label_dict[dominant_label]
            dominant_label_avg_conf = dominant_label_conf / dominant_label_count
            
            contour_conf[contour_idx] = dominant_label_conf
            contour_avg_conf[contour_idx] = dominant_label_avg_conf
            contour_label[contour_idx] = dominant_label
            contour_label_count[contour_idx] = dominant_label_count

        return contour_conf
    
    def filter_outlier_contours(self, lang, contours, contour_conf):
        """Filter outlier contours considering their size and confidence."""
        # sort contour that satisfies following equation: 'min_area < area(contour) < max_area'
        filtered_contour_list = []
        filtered_contour_conf_list = []
        for contour_idx, contour in enumerate(contours):
            if cv2.contourArea(contour) > self.min_area and cv2.contourArea(contour) < self.max_area:
                contour = np.squeeze(contour) # (N, 1 ,2) -> (N, 2)
                filtered_contour_list.append(contour)
                filtered_contour_conf_list.append(contour_conf[contour_idx])
        return filtered_contour_list, filtered_contour_conf_list
    
    def get_landmark_coordinate(self, landmark, direction=None): 
        """Get navgiable coordiante from given single query. overall process is 1)relabeling with confidence weighting and 
        2)find the most confident contour.
        """
        lang = mp3dcat

        if not landmark in lang:
            lang.insert(0, landmark)

        pixel_label, pixel_confidence = self.get_pixel_label_and_confidence(lang)
        self.set_seg(lang, pixel_label)
        # contours = self.get_contour_from_seg(self.seg)
        # pixel_label = self.relabel_contours(contours, pixel_label, pixel_confidence)

        # find all contours of the subgoal
        subgoal = landmark
        subgoal_seg = self.get_subgoal_seg(lang.index(landmark), pixel_label)
        contours = self.get_contour_from_seg(subgoal_seg)
        
        contour_conf = self.get_contours_conf(contours, pixel_label, pixel_confidence)
        
        # sort contour that satisfies following equation: 'min_area < area(contour) < max_area'
        filtered_contour_list, filtered_contour_conf_list = self.filter_outlier_contours(lang, contours, contour_conf)
        
        # raise error for unappropraiate subgoal input
        if len(filtered_contour_list) == 0:
            print(f"Can't detect {subgoal}, since any of {subgoal}'s contours has not appropriate size to be used as a landmark.")
            return
        
        # variables to find max_conf contour
        max_conf_contour_idx = -1
        max_conf_navigable_point = None
        max_conf = 0
        already_arrived = True
        collision_free = False
        
        if len(self.subgoal_dict) == 0:
            prev_coord = self.robot_pixel_pose
        else:
            prev_coord = self.subgoal_dict[list(self.subgoal_dict)[-1]]
        
        # if multiple candidate contours exist, find the most confident contour by comparing confidence of each contour.
        for contour_idx, contour in enumerate(filtered_contour_list): 
            navigable_point, collision = self.find_collision_free_point(contour, prev_coord, direction)
            # compute euclidean distance from agent's current xy & i.d point's xy
            if not collision:
                collision_free = True
                distance = self.get_euclidean_distance(prev_coord, [navigable_point])
                conf = filtered_contour_conf_list[contour_idx]
                if conf > max_conf and distance > self.goal_arrived_thresh:       
                    max_conf = conf             
                    already_arrived = False
                    max_conf_contour_idx = contour_idx
                    max_conf_navigable_point = navigable_point
        
        # save contour to draw
        self.contour_draw_list.append(filtered_contour_list[max_conf_contour_idx])

        if not collision_free:
            print(f"Detect {subgoal}, but robot can't go to {direction} side of {subgoal}.")
            return
        
        if already_arrived:
            print(f"Detect {subgoal}, but robot has already arrived {subgoal}.")
            return

        self.subgoal_dict[subgoal] = max_conf_navigable_point
        
        # save contour to draw
        self.contour_draw_list.append(filtered_contour_list[max_conf_contour_idx])

    def get_coordinate_between_two_landmarks(self, landmark1, landmark2): 
        """Get navgiable coordiante between two given landmark. overall process is 1)relabeling with confidence weighting and 
        2)find the most confident contours. 3) find the navigable point between two landmark.
        """
        # lang = [landmark1, landmark2]
        lang = mp3dcat

        if not landmark1 in lang:
            lang.insert(0, landmark1)
        if not landmark2 in lang:
            lang.insert(1, landmark2)

        pixel_label, pixel_confidence = self.get_pixel_label_and_confidence(lang)
        self.set_seg(lang, pixel_label)
        # contours = self.get_contour_from_seg(self.seg)
        # pixel_label = self.relabel_contours(contours, pixel_label, pixel_confidence)
        max_conf_contours = []
        
        # find all contours of the subgoal
        for landmark in [landmark1, landmark2]:
            subgoal = landmark
            subgoal_seg = self.get_subgoal_seg(lang.index(subgoal), pixel_label)
            contours = self.get_contour_from_seg(subgoal_seg)
            
            contour_conf = self.get_contours_conf(contours, pixel_label, pixel_confidence)
            
            # sort contour that satisfies following equation: 'min_area < area(contour) < max_area'
            filtered_contour_list, filtered_contour_conf_list = self.filter_outlier_contours(lang, contours, contour_conf)
            
            # raise error for unappropraiate subgoal input
            if len(filtered_contour_list) == 0:
                print(f"Can't find subgoal between {lang[1]} and {lang[2]}, since {subgoal} is not found.")
                return
            
            max_conf_contour_idx = -1
            max_conf = 0
            for contour_idx, contour in enumerate(filtered_contour_list): 
                conf = filtered_contour_conf_list[contour_idx]
                if conf > max_conf:         
                    max_conf = conf
                    max_conf_contour_idx = contour_idx
            
            max_conf_contours.append(filtered_contour_list[max_conf_contour_idx])
        navigable_point, collision_free = self.find_point_between_two_landmarks(max_conf_contours[0], max_conf_contours[1])
        
        if collision_free:
            key = f'between_{landmark1}_and_{landmark2}'
            self.subgoal_dict[key] = navigable_point
        else:
            print(f"Can't find subgoal between {landmark1} and {landmark2}, since obstacle exists.")
            return


if __name__ == '__main__':
    args = parse_configs()
    scf = SubgoalCoordinateFilter(args)
    
    for lang in args.lang:
        scf.get_landmark_coordinate(lang)
    
    if args.vis:    
        _, ax = plt.subplots()
        cv2.drawContours(scf.seg, scf.contour_draw_list, -1, (255, 0, 0), 1, cv2.LINE_AA)
        
        for i, (subgoal, coord) in enumerate(scf.subgoal_dict.items()):
            circle = plp.Circle((int(coord[0]), int(coord[1])), 1, color='blue', fill=True, linewidth=3)
            ax.add_patch(circle)
            ax.text(int(coord[0]) + 10, int(coord[1]) - 10, f'{i}:{subgoal}', ha='center', va='center', fontfamily='sans-serif',fontsize=10, color='blue')
            ax.text(int(coord[0]) + 10, int(coord[1]) + 10, f'({coord[0]}, {coord[1]})', ha='center', va='center', fontfamily='sans-serif',fontsize=7, color='blue')

        ax.axis('off')
        ax.imshow(scf.seg)
        plt.legend(handles=scf.patches, loc='upper left', bbox_to_anchor=(1., 1), prop={'size': 10})
        plt.title("Subgoal Coordinate Visualized Map")
        plt.show()
        print(scf.subgoal_dict)
    else:
        print(scf.subgoal_dict)
         
