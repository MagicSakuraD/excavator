#!/bin/bash

# 启动 ROS2 H.264 摄像头发布节点（Jetson/Orin 硬件编码）

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LOG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)/logs"
mkdir -p "$LOG_DIR"

DEVICE="${DEVICE:-/dev/video0}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-480}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-4000000}"
TOPIC="${TOPIC:-/camera_front_wide}"

LOG_FILE="$LOG_DIR/ros2-h264-camera.log"
PID_FILE="$LOG_DIR/ros2-h264-camera.pid"

if [ -z "$ROS_DISTRO" ]; then
  echo "❌ 请先 source ROS2 环境，例如: source /opt/ros/humble/setup.bash"
  exit 1
fi

if [ ! -e "$DEVICE" ]; then
  echo "❌ 摄像头设备不存在: $DEVICE"
  ls -l /dev/video* || true
  exit 1
fi

if [ -f "$PID_FILE" ] && ps -p $(cat "$PID_FILE") > /dev/null 2>&1; then
  echo "⚠️  进程已在运行 (PID: $(cat "$PID_FILE"))"
  exit 0
fi

echo "🚀 启动 ROS2 H.264 摄像头发布: $DEVICE ${WIDTH}x${HEIGHT}@${FPS} bitrate=$BITRATE topic=$TOPIC"

nohup python3 "$SCRIPT_DIR/ros2_h264_camera_publisher.py" \
  --device "$DEVICE" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --fps "$FPS" \
  --bitrate "$BITRATE" \
  --topic "$TOPIC" \
  > "$LOG_FILE" 2>&1 &

PID=$!
echo $PID > "$PID_FILE"
sleep 1

if ps -p $PID > /dev/null 2>&1; then
  echo "✅ 已启动 (PID: $PID)；日志: $LOG_FILE"
else
  echo "❌ 启动失败，请查看日志: $LOG_FILE"
  rm -f "$PID_FILE"
  exit 1
fi



