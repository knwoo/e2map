#!/bin/bash
WATCH_DIR="./images"
rm ./images/* # script doesn't start when images are already inside images/

LAST_STATE=$(ls -A "$WATCH_DIR")

while true; do
    CURRENT_STATE=$(ls -A "$WATCH_DIR")

    if [ -z "$CURRENT_STATE" ]; then
        echo "No special events confirmed"
    else
        if [ "$LAST_STATE" != "$CURRENT_STATE" ]; then
            echo "New special event confirmed"
            
            sleep 3
            
            python event_descriptor.py --model gpt4o

            if [ $? -eq 0 ]; then
                python emotion_evaluator.py
            else
                echo "Error in event_descriptor"
            fi
        fi
    fi

    LAST_STATE=$CURRENT_STATE

    sleep 1
done
