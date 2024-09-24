#!/bin/bash

prompt=${1:-'code'}
model=${2:-'gpt-4o'}

streamlit run ./goal_selector_web.py -- --llm gpt  --prompt $prompt --gpt $model
