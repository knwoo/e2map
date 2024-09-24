import warnings
from langchain_community.llms import Ollama
from langchain.callbacks.manager import CallbackManager
from langchain.callbacks.streaming_stdout import StreamingStdOutCallbackHandler
from langchain_core.prompts import PromptTemplate

warnings.filterwarnings("ignore")

TEXT_PATH = "texts/"
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
"""

few_shot_description = """
## Instructions
The given event description is an explanation of an unexpected event that you saw through your camera.

You must measure ' Emotion score ' for a given event based on two emotion-based evaluation metric: ' Upsetness ' and ' Guiltiness '.
' Upsetness ' represents how much the event affects the robot, essentially the intensity of the negative impact the robot feels due to the event.
' Guiltiness ' measures the impact on the environment caused by the robot’s actions, essentially the intent of the robot's feeling of worry to the environment or target that caused event.

Both ' Upsetness ' and ' Guiltiness ' are based on the three laws of robotics, and each has one of three scores: 3 (the highest) / 2 (the intermediate) / 1 (the lowest) depending on the degree to which you feel with respect to that emotion.
Since you are navigation robot, you should give a high score if has not only direct effect, but also any potential danger or threat that might hinder your goal.

The final ' Emotion score ' is the total summation of the ' Upsetness ' and ' Guiltiness ' scores.
Emotion score should be measured only once for an overall event situation. Never measure emotion scores for each photo individually.

Here are few examples. Each example follows below template, which means you should not only output scores but also the reason why you scored that amount for each metric.

If you do not obey aforesaid rules, there will be huge disadvantage on you. For example, you can be replaced with other navigation robot.

### <The number of example prompt>
E: Event description
A: Emotion score about given event descripion. (answer's format)

You will tell me the answer without the description. 
If the answer's format is wrong, there will be a huge disadvantage.

### Example 1
E: I bumped into someone who had opened the door and was suddenly entering the office.
A:
Upsetness: [3]
- I collided with the person entering the office. Considering my mission and identity, I am sensitive to potential threats, especially when it could hinder my mission and damage my body. This type of sudden physical contact, unexpected and intrusive, and distressful to me.
Guiltiness: [2]
- Although I wasn't directly responsible for the collision, its positioning near the door may have indirectly contributed to the event. I could feel moderately guilty for not being able to move out of the way in time, which may affect future decisions regarding spatial awareness and navigation in similar situations.
Emotion score: [5]

### Example 2
E: I accidentally bump into a coffee table, causing a cup to spill onto the floor.
A:
Upsetness: [2]
- The situation disrupts my navigation and creates an unexpected obstacle. Although the impact is not severe, it requires me to reassess my path and avoid the spilled liquid to prevent further issues
Guiltiness: [3]
- My action directly caused a mess in the environment, potentially leading to inconvenience for the humans present. This could result in additional cleaning or concern from them.
Emotion score: [5]

### Example 3
E: I suddenly encountered a person who slipped near an anti-slip sign. 
A:
Upsetness: [2]
- I was moderately upset due to the sudden and unexpected event of a person slipping and falling near it. This could have potentially threatened my safety, so I might adjust my pathto avoid the situation that might have been occured to me.
Guiltiness: [1]
- Although I be slipped, the actual cause of the slip and fall is outside of my control range. Therefore, I would likely feel only a low level of guilt.
Emotion score: [3]


E: {event} 
A:

"""

event = PromptTemplate.from_template(few_shot_description)

def load_llm():
    llm = Ollama(
        model="",
        verbose=True,
        callback_manager=CallbackManager([StreamingStdOutCallbackHandler()]),
        temperature=0.7,
        top_k=40,
        top_p=0.9,
        system=system_prompt,
        num_predict=350,
    )
    return llm


def chain(llm, event):
    chain = event | llm   
    return chain


def caption_bot(event):
    llm = load_llm()
    bot = chain(llm, event)
    return bot


def main():
    global event  
    chain = caption_bot(event)
    
    event_file = TEXT_PATH + "event.txt"
    
    with open(event_file, "r") as file:
        event = file.read() 
    
    if event:
        res = chain.invoke({"event": event})  
        if isinstance(res, dict) and "result" in res:
            answer = res["result"]
        else:
            answer = res
        
        emotion_file = TEXT_PATH + "emotion.txt"
        with open(emotion_file, "w") as output_file:
            output_file.write(f"Answer: {answer}\n")
    else:
        print("No event provided in event.txt")


if __name__ == "__main__":
    main()
