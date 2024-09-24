#!/usr/bin/python3
import os
import os.path as osp
import sys
import time
import requests

exp_pkg_dir = osp.dirname(osp.dirname(osp.abspath(__file__)))
src_dir = osp.dirname(exp_pkg_dir)
foundations_pkg_dir = osp.join(src_dir, 'foundations')
sys.path.append(src_dir)

import rospy
from langchain_community.llms import Ollama
from std_msgs.msg import Bool, String

from foundations.utils.prompt import get_code_generation_shot, get_landmark_generation_shot_cot


class GoalSelectorWrapper():
    def __init__(self):
        rospy.init_node('goal_selector_node', anonymous=True)
        scenario_path = osp.join(exp_pkg_dir, 'scenarios')
        scenario = rospy.get_param('/scenario')
        self.scenario_commands_path = osp.join(scenario_path, f"{scenario}.txt")
        
        self.prompt_type = rospy.get_param('/prompt_type')
        self.llm_type = rospy.get_param('/llm_type')
        self.scenario_done_topic = rospy.get_param('/scenario_done_topic')
        self.landmarks_text_topic = rospy.get_param('/landmarks_text_topic')
        self.landmarks_list_path = rospy.get_param(rospy.get_name() + '/landmarks_list_path')
        self.gpt_model = None
        self.ollama_model = None

        self.scenario_done_flag = Bool(False)
        self.landmarks_text_pub = rospy.Publisher(self.landmarks_text_topic, String, queue_size=10)
        self.scenario_done_pub = rospy.Publisher(self.scenario_done_topic, Bool, queue_size=10)
        self.reset_topic_name = rospy.get_param('/reset_topic')
        self.map_init_topic_name = rospy.get_param('/map_init_topic')
        self.reset_sub = rospy.Subscriber(self.reset_topic_name, Bool, self.reset_callback)
        self.map_init_sub = rospy.Subscriber(self.map_init_topic_name, Bool, self.map_init_callback)
        self.reseted = False
        self.map_init = False

        self.count = 0

        self.commands = None
        self.num_commands = 0
        self.command_idx = 1
        
        self._load_commands()
        self._load_llm_model()
        
    def _load_commands(self):
        with open(self.scenario_commands_path, 'r') as f:
            self.commands = f.readlines()
            self.num_commands = len(self.commands)
        
    def _load_llm_model(self):
        if self.llm_type == 'gpt':
            self.gpt_model = rospy.get_param(rospy.get_name() + '/gpt_model')
        elif self.llm_type == 'ollama':
            self.ollama_model = rospy.get_param(rospy.get_name() + '/ollama_model')
    
    def _local_llm(self, instruction, model):
        assert model, "Ollama model should be defined in class initializer."
        llm = Ollama(model=model)
        message_content = llm.invoke(instruction)
        return message_content

    def _gpt_llm(self, instruction, model):
        assert model, "GPT model should be defined in class initializer."
        api_key = os.getenv('OPENAI_API_KEY')  
        headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
        }
        payload = {
            "model": model,
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": instruction
                        }
                    ]
                }
            ],
            "max_tokens": 300
        }
        response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
        message_content = response.json()['choices'][0]['message']['content']
        return message_content
    
    def reset_callback(self, msg):
        if msg.data:
            self.reseted = msg.data
    
    def map_init_callback(self, msg):
        self.map_init = msg.data

    def publish_scenario_done(self):
        self.scenario_done_flag = Bool(True)
        self.scenario_done_pub.publish(self.scenario_done_flag)
        self.scenario_done_flag = Bool(False)
    
    def publish_landmarks(self, landmarks: str):
        self.landmarks_text_pub.publish(landmarks)

    def select_goal(self, command):
        # make instruction
        if self.prompt_type == 'code':
            instruction = get_code_generation_shot(command)
        elif self.prompt_type == 'landmark':
            instruction = get_landmark_generation_shot_cot(command)
        
        # feedforward
        if self.llm_type == 'gpt':
            response = self._gpt_llm(instruction, self.gpt_model)
        elif self.llm_type == 'ollama':
            response = self._local_llm(instruction, self.ollama_model)

        return response

    def save_landmark_to_txt(self, path, landmarks):
        with open(path, 'w') as f:
            f.write(landmarks)
    
    def parse_subgoal(self):
        if len(self.commands) != 0:
            command = self.commands.pop(0)
            
            landmarks = self.select_goal(command)
            self.publish_landmarks(str(landmarks))
            print(f"Command {self.command_idx}: {command}") 
            print(f"Parsed result: {landmarks}")
            
            self.save_landmark_to_txt(self.landmarks_list_path, landmarks)
            
            self.command_idx += 1
        else:
            self.publish_scenario_done()
        
        
if __name__ == '__main__':
    gsw = GoalSelectorWrapper()
    gsw.reseted = True
    scenario_commands_path = gsw.scenario_commands_path
    landmarks_list_path = gsw.landmarks_list_path
    commands = None
    num_commands = 10
    
    # make directory to save landmark
    text_dir_name = "text"
    if not osp.exists(osp.join(foundations_pkg_dir, text_dir_name)):
        os.mkdir(osp.join(foundations_pkg_dir, text_dir_name))
    
    if osp.isfile(landmarks_list_path):
        os.remove(landmarks_list_path)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if gsw.map_init:
            if gsw.reseted:
                gsw.parse_subgoal()
                gsw.reseted = False
            
            if osp.isfile(landmarks_list_path):
                print("command in progress..")
                time.sleep(5)
        
        rate.sleep()