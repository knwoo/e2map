#!/bin/bash

prompt=${1:-'code'}
model=${2:-'llama3:8b'}

streamlit run ./goal_selector_web.py -- --llm ollama --prompt $prompt --ollama $model
