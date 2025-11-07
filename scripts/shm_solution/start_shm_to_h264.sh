#!/bin/bash
# Start Shared Memory to H.264 stdout (pure GStreamer)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source ROS2 environment is not required here, but harmless
source /opt/ros/humble/setup.bash 2>/dev/null || true

# PID & LOG
PID_FILE="$WORKSPACE_DIR/logs/shm_to_stdout.pid"
LOG_FILE="$WORKSPACE_DIR/logs/shm_to_stdout.log"
mkdir -p "$WORKSPACE_DIR/logs"

# Kill existing
if [ -f "$PID_FILE" ]; then
    OLD_PID=$(cat "$PID_FILE")
    if ps -p "$OLD_PID" > /dev/null 2>&1; then
        echo "Killing existing shm_to_stdout (PID: $OLD_PID)"
        kill "$OLD_PID" 2>/dev/null || true
        sleep 1
    fi
    rm -f "$PID_FILE"
fi

# Parameters (can override via env)
export SHM_PATH="${SHM_PATH:-/dev/shm/isaac_rgb_buffer}"
export WIDTH="${WIDTH:-1280}"
export HEIGHT="${HEIGHT:-720}"
export FPS="${FPS:-20}"
export BITRATE_KBPS="${BITRATE_KBPS:-4000}"

# Start
echo "Starting shm_to_stdout.py (SHM_PATH=$SHM_PATH, ${WIDTH}x${HEIGHT}@${FPS}, ${BITRATE_KBPS}kbps)"
python3 "$SCRIPT_DIR/shm_to_stdout.py" > "$LOG_FILE" 2>&1 &

echo $! > "$PID_FILE"
echo "shm_to_stdout started (PID: $(cat $PID_FILE))"
echo "Logs: $LOG_FILE"

