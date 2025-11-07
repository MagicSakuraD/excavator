#!/bin/bash
# Start ROS2 to Shared Memory Bridge
# This script launches the pure ROS2 node that writes to shared memory

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# PID file
PID_FILE="$WORKSPACE_DIR/logs/ros_to_shm.pid"
LOG_FILE="$WORKSPACE_DIR/logs/ros_to_shm.log"

# Create logs directory
mkdir -p "$WORKSPACE_DIR/logs"

# Kill existing process if running
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    if ps -p "$OLD_PID" > /dev/null 2>&1; then
        echo "Killing existing ros_to_shm process (PID: $OLD_PID)"
        kill "$OLD_PID"
        sleep 1
    fi
    rm -f "$PID_FILE"
fi

# Clean up any existing shared memory file
rm -f /dev/shm/isaac_rgb_buffer

# Start the ROS2 to SHM bridge
echo "Starting ROS2 to Shared Memory Bridge..."
echo "Logs: $LOG_FILE"

python3 "$SCRIPT_DIR/ros_to_shm.py" \
    --ros-args \
    -p input_topic:=/stitched_image \
    -p shm_path:=/dev/shm/isaac_rgb_buffer \
    -p width:=1280 \
    -p height:=720 \
    > "$LOG_FILE" 2>&1 &

# Save PID
echo $! > "$PID_FILE"
echo "ROS2 to SHM bridge started (PID: $(cat $PID_FILE))"
echo "Monitoring log: tail -f $LOG_FILE"

