# ref: https://arxiv.org/abs/2207.04429
def get_landmark_generation_shot(instr):
    landmark_generation_shot = f"""
        Look for a library, after taking a right turn next to a statue.
        
        Landmarks:
        1. a statue
        2. a library
        
        Look for a statue. Then look for a library. Then go towards a pink house.
        
        Landmarks:
        1. a statue
        2. a library
        3. a pink house
        
        {instr}
        
        Landmarks:
        1. 
        """
    return landmark_generation_shot


# ours
def get_landmark_generation_shot_cot(instr):
    landmark_generation_shot_cot = f"""
        Please Parse language instruction into a series of landmarks. 
        I'll give you the Instruction like below. 
        I'll give you just instruction like I:[instruction]. 
        You'll think about instruction like T:[thought]. 
        Finally you'll answer like [landmarks].
        Here are few shots. 
        
        some sentences might include directional information or phrases like below list. You should not parse thsese as a landmark.
        
        Left-hand, Leftward, Sinistral, To the left, On the left side, Leftward side, Left-hand side
        Lower, Base, Foot, Underside, Bottommost, Nether, Bottom edge, Underneath, Beneath, Bottom part
        Amid, Among, In the middle of, Surrounded by, In the midst of, Betwixt, Intermediate, Midway, Intervening, Centrally located within
        Right-hand, Rightward, To the right, On the right side, Right-hand side, Rightward side, Rightmost, Right flank
        Upper, Uppermost, Topmost, Peak, Apex, Summit, Crest, Highest, Above, Topside
        
        #1
        I: Go to the chair and then go to another chair. 
        T: I have to head to the chair first. That's why the first landmark will be a chair. Next time I have to head to another chair. Then the second landmark will be a chair. So landmarks will be chair, chair.    
        ["chair", "chair"]
        
        #2
        I: Approach the window in front, go leftside of the television, and finally go to the bottom side of the oven. 
        T: The instruction begins with go to the window, which is identified as the first landmark. then the instruction specifies going to the television. Finally, the instruction gives me to arrive at oven.
        ["window", "television", "oven"]  
        
        #3 
        I: Approach the window in front, turn right and go to the television, and finally go by the oven in the kitchen. 
        T: The initial direction is to approach the window in front, making the window the first landmark. After reaching the window, turning right leads to the television, which becomes the next landmark. The instruction ends with going by the oven in the kitchen, indicating the oven AND kitchen as the final landmarks. Therefore, the landmarks in sequence are window, television, oven, kitchen.    
        ["window", "television", "oven", "kitchen"]
        
        #4
        I: Walk to the plant first, turn around and come back to the table, go further into the bedroom, and stand next to the bed. 
        T: Starting with walking to the plant, it is identified as the first landmark. Turning around to come back to the table positions the table as the next landmark. Proceeding further into the bedroom indicates the bedroom itself as a landmark. Finally, standing next to the bed makes the bed the last landmark. Therefore, the landmarks in order are plant, table, bedroom, bed.
        ["plant", "table", "bedroom", "bed"]
        
        #5
        I: Go by the stairs, go to the room next to it, approach the book shelf and then go to the table in the next room. 
        T: The instruction begins with going by the stairs, which is identified as the first landmark. Following the stairs, the instruction specifies going to the room next to it, making this room the second landmark. The next step is to approach the bookshelf within that room, establishing the bookshelf as the third landmark. The final part of the instruction involves going to the table in the next room, suggesting two more landmarks: the table and the implication of entering another room, which would be the final landmark.    
        ["stairs", "room", "bookshelf", "table"]
        
        #6
        I: Go front left and move to the table, then turn around and find a cushion, later stand next to a column before finally navigate to any appliances. 
        T: The directions start with moving front left to reach the table, making the table the first landmark. After reaching the table, turning around to find a cushion points to the cushion as the next landmark. The instruction then suggests standing next to a column, identifying the column as the subsequent landmark. The final step is to navigate to any appliances, which generalizes the last landmark to be any type of appliance. Therefore, the sequence of landmarks based on the given instructions is table, cushion, column, and appliances.
        ["table", "cushion", "column", "appliances"]
        
        #7
        I: Navigate to the right side of sofa and go straight to chair. Finally, your goal is the painting. 
        T: The instruction begins with going by the sofa, which is identified as the first landmark. then the instruction specifies going to the chair. Finally, the instruction gives me to arrive at painting.
        ["sofa", "chair", "painting"]    
        
        #8
        I: Approach the central fountain, turn left, and find the nearest bench.
        T: The instruction begins with approaching the central fountain, establishing it as the first landmark. After reaching the central fountain and turning left, the next step is to find the nearest bench, which becomes the subsequent landmark. Thus, the sequence of landmarks based on the instruction given is the central fountain followed by the nearest bench.
        ["fountain", "bench"]
        
        #9
        I: Go leftward to the table, then find a cushion, later stand between a sofa and a vase. 
        T: The instruction begins with approaching table first. Then, it gives direction to cushion, and finally sofa and vase sequentially.
        ["table", "cushion", "sofa", "vase"] 
        
        Please just answer in only words. and your answer should be encapsulated with list.
        
        I: {instr}
        A: 
        """
    return landmark_generation_shot_cot


# ours
def get_code_generation_shot(instr): 
    code_generation_shot = f"""
        Please parse given language instruction into a corresponding code snippet. 
        Here are few examples. 
        Each example follows below template.
        
        # <The number of example prompt>
        I: language instruction (each instruction might end with dot or without dot. So do not think about it that much please.)
        C: ["code snippet"] 
        
        Each code line can be fall into 6 kinds: <self.go_to(), self.go_left_of(), self.go_right_of(), self.go_top_of(), self.go_bottom_of(), self.go_between()>
        
        Without self.go_to(), each code line includes directional information such as left and right. Hence, there might be other synonyms that represents aforementioned canonical direction.

        Therefore, I will give synonym examples to help you. When you see below kinds of synonyms, you can just map it into aforementioned canonical direction.
        
        Left-hand, Leftward, Sinistral, To the left, On the left side, Leftward side, Left-hand side -> left
        Lower, Base, Foot, Underside, Bottommost, Nether, Bottom edge, Underneath, Beneath, Bottom part -> bottom
        Amid, Among, In the middle of, Surrounded by, In the midst of, Betwixt, Intermediate, Midway, Intervening, Centrally located within -> between
        Right-hand, Rightward, To the right, On the right side, Right-hand side, Rightward side, Rightmost, Right flank -> right
        Upper, Uppermost, Topmost, Peak, Apex, Summit, Crest, Highest, Above, Topside -> upper
        
        Also, to ease your burden, each code in code snippet has a prefix: self.
        
        #1
        I: Go to the chair and then go to sofa. 
        C: ["self.go_to('chair')", "self.go_to('sofa')"]
        
        #2
        I: Navigate to the right side of sofa and go straight to chair. Finally, your goal is the painting. 
        C: ["self.go_right_of('sofa')", "self.go_to('chair')", "self.go_to('painting')"]    
        
        #3 
        I: Approach the window in front, go leftside of the television, and finally go to the bottom side of the oven. 
        C: ["self.go_right_of('sofa')", "self.go_left_of('television')", "self.go_bottom_of('oven')"] 
        
        #4
        I: Walk to the plant first, turn around and come back to the table, go further into the bedroom, and stand top side of the bed. 
        C: ["self.go_to('plant')", "self.go_to('table')", "self.go_to('bedroom')", "self.go_top_of('bed')"] 

        #5
        I: Go by the stairs, approach on the right side of the book shelf and then go to the table in the next room. 
        C: ["self.go_to('stairs')", "self.go_right_of('shelf')", "self.go_to('table')"] 
        
        #6
        I: Move to the bed.
        C: ["self.go_to('bed')"] 

        #7
        I: Go leftward to the table, then find a cushion, later stand between a sofa and a vase. 
        C: ["self.go_left_of('bed')", "self.go_to('cushion')", "self.go_between('sofa','vase')"] 
        
        #8
        I: Go to the vending machine, and then turn to go to the upper side of bench and stop at the near wall.
        C: ["self.go_to('vending machine')", "self.go_top_of('bench')", "self.go_to('wall')" ] 

        #9
        I: Approach to the the underside of printer, then stop leftside of the shelf, and finally stop at the between chair and desk.
        C: ["self.go_bottom_of('printer')", "self.go_left_of('shelf')", "self.go_between('chair','desk')"]

        #10
        I: Straight to the nearset refrigerator, then go to the top side of microwave. Finally, go straight to the toaster. 
        C: ["self.go_to('refrigerator')", "self.go_top_of('microwave')", "self.go_to('toaster')"]   

        #11
        I: Move to the waste bin, then go to the nearest whiteboard. Then you should stop in the middle of chair and wall. 
        C: ["self.go_to('waste bin')", "self.go_to('whiteboard')", "self.go_between('chair','wall')"] 

        #12
        I: Go in the midway of table and refrigerator.
        C: ["self.go_between('table','refrigerator')"] 

        Like aforementioned examples, you must tell me the list that contains the code snippet.
        Please answer just Answer in only words.
        Your answer should follow this format: ["<the first code line>", "<the second code line>", "<the third code line>", .....].
        Do not tell me OTHER SENTENCE except the answer.
        If the answer's format is wrong, there will be a huge disadvantage.
        
        I: {instr}
        C: 
        """    
    return code_generation_shot
