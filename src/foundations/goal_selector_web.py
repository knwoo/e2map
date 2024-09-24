import os
import requests
import argparse

import streamlit as st
from langchain_community.llms import Ollama

from utils.prompt import get_code_generation_shot, get_landmark_generation_shot_cot


def local_llm(instruction, model):
    """Function to set up a local LLM using Ollama"""
    select_ollama = model
    llm = Ollama(model=select_ollama)
    message_content = llm.invoke(instruction)
    return message_content


def gpt_llm(instruction, model):
    """Function to set LLM using GPT"""
    # OpenAI API key
    api_key = os.getenv('OPENAI_API_KEY')  
    gpt_model = model
    headers = {
    "Content-Type": "application/json",
    "Authorization": f"Bearer {api_key}"
    }
    payload = {
        "model": gpt_model,
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


def main():
    parser = argparse.ArgumentParser(description="web interface for langnav")
    parser.add_argument("--llm", default='gpt', help="Choose how to use llm")
    parser.add_argument("--ollama", default='llama3:70b', help="Choose Ollama model")
    parser.add_argument("--gpt", default='gpt-4o', help="Choose GPT model")
    parser.add_argument("--prompt", default='code', choices=['code', 'landmark'], help="Choose what kinds of prompt for selecting goals.")
    args = parser.parse_args()

    if not os.path.exists('./text'):
        os.mkdir('./text')
    
    text_dir_path = os.path.abspath("./text")
    landmark_filename = 'landmarks_list.txt'
    landmarks_list_path = os.path.join(text_dir_path, landmark_filename)
    if os.path.isfile(landmarks_list_path):
        os.remove(landmarks_list_path)
    
    # Initialize a session state variable to store instruction-answer pairs if it doesn't exist
    if 'instruction_pairs' not in st.session_state:
        st.session_state['instruction_pairs'] = []

    with st.form('my_form'):
        # Display previous instructions and their responses
        if st.session_state['instruction_pairs']:
            st.subheader('Previous Instructions and Responses')
            for instruction, response in st.session_state['instruction_pairs']:
                st.text(f"Instruction: {instruction}")
                st.text(f"Response: {response}")
                st.write("---")  

        # Text area for new instruction
        text = st.text_area('Enter Instruction:', key="new_instruction")
        submitted = st.form_submit_button('Submit')

        if submitted:
            # Get the response for the new instruction
            if args.prompt == 'code':
                instruction = get_code_generation_shot(text)
            elif args.prompt == 'landmark':
                instruction = get_landmark_generation_shot_cot(text)

            select_llm = args.llm
            if select_llm == 'gpt':
                response = gpt_llm(instruction, args.gpt)
            elif select_llm == 'ollama':
                response = local_llm(instruction, args.ollama)
            else:
                response = "Unsupported method"

            # Add the new instruction and response to the session state
            st.session_state['instruction_pairs'].append((text, response)) 
            
            # Display the response
            st.info(response) 

            # Save the result
            with open(landmarks_list_path, "w") as file:
                file.write(response)
           
           
# !streamlit run app.py
if __name__ == '__main__':
    main()

        