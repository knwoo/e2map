#!/usr/bin/env python
import os
import os.path as osp
import csv
import ast
from datetime import datetime

import rospy
from std_msgs.msg import Bool, String

exp_dir = osp.dirname(osp.dirname(osp.abspath(__file__)))
save_path = osp.join(exp_dir, 'result')
    

class SuccessCounter():
    def __init__(self):
        rospy.init_node('success_counter_node', anonymous=True)
        self.method = rospy.get_param('/method')
        self.success_topic = rospy.get_param('/success_topic')
        self.collision_detection_topic = rospy.get_param('/collision_detection_topic')
        self.subgoal_text_topic = rospy.get_param('/subgoal_text_topic')
        self.reset_topic = rospy.get_param('/reset_topic')
        self.scenario_done_topic = rospy.get_param('/scenario_done_topic')
        self.landmarks_text_topic = rospy.get_param('/landmarks_text_topic')
        self.scenario = rospy.get_param('/scenario')
        self.prompt_tpye = rospy.get_param('/prompt_type')
        self.event_topic = rospy.get_param("/collision_detection_topic")
        self.landmarks_list_path = rospy.get_param(rospy.get_name() + '/landmarks_list_path')

        self.success_sub = rospy.Subscriber(self.success_topic, Bool, self.success_callback)
        self.reset_sub = rospy.Subscriber(self.reset_topic, Bool, self.reset_callback)
        self.event_sub = rospy.Subscriber(self.event_topic, Bool, self.event_callback)
        self.scenario_flag_sub = rospy.Subscriber(self.scenario_done_topic, Bool, self.scenario_flag_callback)
        self.landmarks_text_sub = rospy.Subscriber(self.landmarks_text_topic, String, self.landmarks_text_callback)


        self.landmarks_list = None
        self.success = False
        self.collision = False

        self.total_result = []
        self.keys = ['subgoal', 'success']
    
    def save_episode_result(self):
        result = {}
        if self.success:
            print("Success saved!")
            result['subgoal'] = self.landmarks_list
            result['success'] = 1
            self.total_result.append(result)
            self.success = False

        elif self.collision:
            print("Failure saved!")
            result['subgoal'] = self.landmarks_list
            result['success'] = 0
            self.total_result.append(result)
            self.collision = False

    def reset_callback(self, msg):
        if msg.data:
            self.save_episode_result()
            self.landmarks_list = None
    
    def event_callback(self, msg):
        if msg.data:
            self.collision = msg.data

    def success_callback(self, msg):
        if msg.data:
            self.success = msg.data
            
    def landmarks_text_callback(self, msg):
        self.landmarks_list = ast.literal_eval(msg.data)
        
    def scenario_flag_callback(self, msg):
        if msg.data:
            if not osp.exists(save_path):
                os.makedirs(save_path)
            
            n_success = sum([result['success'] for result in self.total_result])
            n_episode = len([result['success'] for result in self.total_result])
            sr = n_success / n_episode * 100

            self.total_result.append({'subgoal': "Success rate:", 'success': sr})
            current_time = datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
            
            logfile = osp.join(save_path, f"{self.scenario}_{self.method}_{current_time}.csv")
            with open(logfile, 'w', newline='') as f:
                dict_writer = csv.DictWriter(f, fieldnames=self.keys)
                dict_writer.writeheader()
                dict_writer.writerows(self.total_result)
                
            print(f"Saved {self.scenario}'s result of {self.method}.")
            
            self.total_result.clear()
            rospy.signal_shutdown(f"Terminating {self.method}'s {self.scenario} experiment")

if __name__ == '__main__':
    try:
        sc = SuccessCounter()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():    
            rate.sleep()

    except rospy.ROSInterruptException:
        pass