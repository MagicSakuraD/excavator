#!/bin/bash

# 启动 excavator (ROS2 H.264 Stdout 桥接模式)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BIN_DIR="$PROJECT_ROOT/bin"
LOG_DIR="$PROJECT_ROOT/logs"
mkdir -p "$LOG_DIR"

# 可配置参数
SIGNALING_SERVER="${SIGNALING_SERVER:-ws://111.186.56.118:8090/ws}"
ROS2_IMAGE_TOPIC="${ROS2_IMAGE_TOPIC:-/camera_front_wide}"

LOG_FILE="$LOG_DIR/excavator-bridge.log"
PID_FILE="$LOG_DIR/excavator.pid"

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
  echo "❌ 请先 source ROS2 环境，例如: source /opt/ros/humble/setup.bash"
  exit 1
fi

echo "🔨 编译 excavator ( stdout 桥接模式 )..."
cd "$PROJECT_ROOT"
go build -o "$BIN_DIR/excavator" ./cmd/excavator

# 停止旧进程
if [ -f "$PID_FILE" ] && ps -p $(cat "$PID_FILE") > /dev/null 2>&1; then
  echo "🛑 停止旧进程 $(cat "$PID_FILE") ..."
  kill $(cat "$PID_FILE") 2>/dev/null || true
  sleep 1
fi

echo "🚀 启动 excavator (ros2_h264_bridge)..."
cd "$BIN_DIR"

# 启动 Go 程序，它会作为父进程启动 Python 脚本
nohup ./excavator \
  -signaling "$SIGNALING_SERVER" \
  -ros2-image-topic "$ROS2_IMAGE_TOPIC" \
  -ros2-control-topic "/controls/teleop" \
  > "$LOG_FILE" 2>&1 &

PID=$!
echo $PID > "$PID_FILE"
sleep 2

# Go 程序启动后，Python 脚本会作为子进程被启动
# 我们可以检查一下
CHILD_PID=$(pgrep -P $PID)

if ps -p $PID > /dev/null 2>&1; then
  echo "✅ Go 父进程已启动 (PID: $PID)"
  if [ -n "$CHILD_PID" ] && ps -p $CHILD_PID > /dev/null 2>&1; then
    echo "✅ Python 桥接子进程已启动 (PID: $CHILD_PID)"
  else
    echo "⚠️  Python 桥接子进程似乎未启动，请检查日志: $LOG_FILE"
  fi
  echo "📝 日志文件: tail -f $LOG_FILE"
  echo "🛑 停止: kill $PID"
else
  echo "❌ 启动失败，请查看日志: $LOG_FILE"
  rm -f "$PID_FILE"
  exit 1
fi


