#!/bin/bash
# Start all components on the Excavator (Edge) side
# This launches the ROS2->SHM bridge and the Go WebRTC client (which auto-spawns the encoder)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS2 environment (required for ros_control_stdin.py)
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[INFO] ROS2 environment loaded"
else
    echo "[WARN] ROS2 environment not found at /opt/ros/humble/setup.bash"
    echo "       Control bridge may fail to start"
fi

echo "=========================================="
echo "Starting Excavator-side (ROS->SHM + WebRTC Client)"
echo "=========================================="

LOG_DIR=~/code_ws/excavator/logs
mkdir -p "$LOG_DIR"

# Resolve project root and binary path
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BIN_PATH="$PROJECT_ROOT/bin/excavator"

# Read signaling URL from first argument or env SIGNALING_URL
SIGNALING_URL="${1:-${SIGNALING_URL}}"
if [ -z "$SIGNALING_URL" ]; then
  echo "[ERROR] Missing signaling URL. Usage: $0 ws://<cloud-ip>:8090/ws"
  echo "        or export SIGNALING_URL=ws://<cloud-ip>:8090/ws and run $0"
  exit 1
fi

# Step 1: Start ROS2 to SHM bridge first
echo ""
echo "Step 1: Starting ROS2 -> Shared Memory Bridge..."
"$SCRIPT_DIR/start_ros_to_shm.sh"
sleep 2

# Step 2: Build Go WebRTC client if needed
echo ""
echo "Step 2: Building Go WebRTC client if missing..."
if [ ! -x "$BIN_PATH" ]; then
  (cd "$PROJECT_ROOT" && go build -o "$BIN_PATH" ./cmd/excavator) || { echo "[ERROR] Go build failed"; exit 1; }
else
  echo "Go binary exists: $BIN_PATH"
fi

# Step 3: Start Go WebRTC client (it will spawn shm_to_stdout.py automatically)
echo ""
echo "Step 3: Starting Go WebRTC client (signaling: $SIGNALING_URL) ..."
"$BIN_PATH" -signaling "$SIGNALING_URL" -supabase-url "$SUPABASE_URL" -supabase-key "$SUPABASE_KEY" > "$LOG_DIR/go_webrtc.log" 2>&1 &
GO_PID=$!
echo "Go WebRTC client started with PID: $GO_PID (logs: $LOG_DIR/go_webrtc.log)"

echo ""
echo "=========================================="
echo "All components started! (ROS->SHM + Go WebRTC client)"
echo "=========================================="
echo ""
echo "Monitor logs with:"
echo "  tail -f ~/code_ws/excavator/logs/ros_to_shm.log"
echo "  tail -f ~/code_ws/excavator/logs/go_webrtc.log"
echo ""
echo "Check status:"
echo "  ros2 topic list"
echo "  ls -lh /dev/shm/isaac_rgb_buffer"
echo ""

