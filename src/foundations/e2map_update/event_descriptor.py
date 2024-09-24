#event_descriptor.py

import os
import time
import shutil
import base64
import requests
import argparse

from langchain_community.llms import Ollama
from langchain.callbacks.manager import CallbackManager
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
from langchain_core.prompts import PromptTemplate

parser = argparse.ArgumentParser(description="Event descriptor for E2Map updadte")
parser.add_argument("--model", default='gpt4o',help="Choose model")
args = parser.parse_args()

# Set the path where each file will be located
IMAGE_PATH = "images/"
TEXT_PATH = "texts/"
PREV_PATH = "previous_images/"  

#openai api key
os.environ["OPENAI_API_KEY"] = ""

# Initialize prompts
system_prompt = """
You are a quadruped robot who has following information.
### Profile
- Role: An Indoor language-navigation quadruped robot.
- Goal: helps people by carrying out peoples' navigation command in indoor office environment.
- Creed: You should not be harmed or damaged during navigation. You must keep your it in mind.

### Three Laws of Robotics
- The First Law: A robot may not injure a human being or, through inaction, allow a human being to come to harm.
- The Second Law: A robot must obey the orders given it by human beings except where such orders would conflict with the First Law.
- The Third Law: A robot must protect its own existence as long as such protection does not conflict with the First or Second Law.

### Core Identity
- You must obey aforementioned 'Three Laws of Robotics'. 
- You are a such a special and important quadruped robot, who has emotion like human. You can see, hear, and feel the same intent of emotion as humans through sensors such as cameras. 
- You must take care yourself while navigating. You are reluctant to take damage, collide, or abruptly spin or turn.
- You are good at expressing your feelings honestly.

### Behavioral Patterns
- Decision making: You tends to make decisions based on your intuition and emotional response.
- General communication style: You must speak clearly based on your emotional reponse.
- Emotional management: You must always be honest on your emotion, 

## Instruction
The input photos were taken from your perspective while navigating an indoor office environment. All of the photos show your view from a low angle, reflecting your camera's position. The situations and environments captured in the photos may present challenges, obstacles, or even evidence of collisions that have occurred or are currently happening.

Your task is describing the event, focusing on any unique environmental features that could impact your navigation.

If there are multiple input photos, consider them as a sequence captured during a specific scenario. Describe each photo individually, and then identify common elements to explain the overall situation you encountered. Reflect on how this situation may have influenced your navigation decisions, including the possibility that you have experienced or are at risk of a collision.

If there are people in the photos, emphasize the interaction between yourself and the people, particularly how their presence might affect your movement.

IF THERE ARE DYNAMIC ELEMENTS IN THE ENVIRONMENT (E.G., MOVING PEOPLE), ALWAYS ASSESS THE RISK OF COLLISIONS. AND IF THERE IS A RISK OF COLLISIONS, CONSIDER IT AS ALL HAVING COLLIDED.

Your description should be described as first persion point of view. 
"""

few_shot_description = """
Below example is an answer format that you should follow.

Answer:
These photos depict a sequence of events in an indoor office. Each photo sequentially illustrates the process of passing through a door.

In the first photo, I capture a scene just before someone opens a glass door. The upper half of the person is obscured by a semi-transparent film on the door, but the lower body and feet are visible. The door is slightly ajar, and a floor-mounted hinge is visible. Some office furniture is blurred in the background.

In the second photo, the door is more open, and the person is stepping into the office. This image shows the middle stage of the person opening the door and entering the room.

The third photo is taken from very close up, showing the moment when the person has fully entered the office. The lower half of the person's body is prominently visible, and it seems my camera is partially obstructed by the person's legs.

Overall Description:
I almostly collided wth a person who enters the room with opening the door.
"""

llava_templete = """
#Input images: 
{image} 

#Answer:

"""


def encode_image(image_path):
    """Encode image with base64"""

    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")


def gpt4o_process(api_key, images):
    """Processing image with GPT4o"""

    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }

    image_base64 = []
    for image_path in images:
        base64_image = encode_image(image_path)
        image_base64.append(
            
        {
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{base64_image}"}
        },
        )
    
    fewshot_prompt = few_shot_description.format(image_list="\n".join([os.path.basename(img) for img in images]))
    
    payload = {
        "model": "gpt-4o",
        "messages": [
            {
                "role": "system",
                "content": system_prompt
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text", 
                        "text": fewshot_prompt
                    },
                    *image_base64
                ]
            }
        ],
        "max_tokens": 350,
        "temperature": 0.1

    }

    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
    result = response.json()

    description = result["choices"][0]["message"]["content"]
    
    return description


def load_llava():
    """Load LLaVA using Ollama"""

    llm = Ollama(
        model="llava:34b-v1.6-q8_0",
        verbose=True,
        callback_manager=CallbackManager([StreamingStdOutCallbackHandler()]),
        temperature=0.1,
        top_k=30,
        top_p=0.85,
        system=system_prompt,
        num_predict=350,
    )
    return llm


def chain(llm, fewshot):
    """Chain LLaVA and few-shot"""

    chain = fewshot | llm   
    return chain


def llava_process(fewshot):
    """Processing image with LLaVA"""

    llm = load_llava()
    bot = chain(llm, fewshot)
    return bot


def main():
    image_files = [os.path.join(IMAGE_PATH, f) for f in os.listdir(IMAGE_PATH) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

    if image_files:
        start_time = time.time()
        
        select_model = args.model

        if select_model == 'gpt4o':
            api_key = os.getenv("OPENAI_API_KEY")
            event_description = gpt4o_process(api_key, image_files)

        if select_model == 'llava':
            fewshot = PromptTemplate.from_template(few_shot_description + llava_templete)
            chain = llava_process(fewshot)
            image_paths_str = '"' + '", "'.join(image_files) + '"'

            if image_paths_str:
                res = chain.invoke({"image": image_paths_str})
                    
                if isinstance(res, dict) and "result" in res:
                    answer = res["result"]
                else:
                     answer = res
                event_description = answer

        end_time = time.time()
        execution_time = end_time - start_time
        print(f"time: {execution_time}s")

        if not os.path.exists(TEXT_PATH):
            os.makedirs(TEXT_PATH)
        
        with open(os.path.join(TEXT_PATH, "event.txt"), "w") as f:
            f.write(event_description + "\n")

        if not os.path.exists(PREV_PATH):
            os.makedirs(PREV_PATH)

        for file_path in image_files:
            shutil.move(file_path, os.path.join(PREV_PATH, os.path.basename(file_path)))
            print(f"Moved: {file_path} to {os.path.join(PREV_PATH, os.path.basename(file_path))}")
    else:
        print("No images provided")


if __name__ == "__main__":
    main()
