#!/bin/bash

if [ -f running_pids.txt ]; then
    echo "Killing processes..."
    for pid in $(cat running_pids.txt); do
        kill "$pid" 2>/dev/null && echo "Killed $pid"
    done
    rm running_pids.txt
else
    echo "No running_pids.txt file found."
fi