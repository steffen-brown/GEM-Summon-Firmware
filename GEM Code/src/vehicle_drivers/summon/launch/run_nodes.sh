#!/bin/bash

# Define relative script path
SCRIPT_DIR="../scripts"

# List of script files (in order)
SCRIPTS=(
    "summon_manager.py"
    "lane_detection.py"
    "object_avoidance.py"
    "networking_node.py"
    "arrive.py"
    "exit_parking.py"
    "lf_pid.py"
)

# Array to hold PIDs
pids=()

# Run each script in the background with 2s spacing
for script in "${SCRIPTS[@]}"; do
    echo "Launching $script..."
    python3 "$SCRIPT_DIR/$script" &
    pids+=($!)
    sleep 2
done

# Save all PIDs to a file
echo "${pids[@]}" > running_pids.txt
echo "All scripts launched. PIDs: ${pids[@]}"
