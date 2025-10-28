#!/bin/bash

# 挖掘机程序启动脚本

set -e

cd "$(dirname "$0")/../bin"

# ===== 配置参数 =====
# 修改这里的信令服务器地址
SIGNALING_SERVER="ws://192.168.3.57:8090/ws"

# 摄像头配置
CAMERA_DEVICE="/dev/video0"
VIDEO_WIDTH=640
VIDEO_HEIGHT=480
VIDEO_FPS=30

# ==================

# 检查是否有旧进程
if pgrep -f "excavator.*$SIGNALING_SERVER" >/dev/null 2>&1; then
    echo "⚠️  挖掘机程序已在运行！正在停止旧进程..."
    pkill -f "excavator.*$SIGNALING_SERVER" || true
    sleep 1
fi

# 检查摄像头
if [ ! -e "$CAMERA_DEVICE" ]; then
    echo "❌ 摄像头设备不存在: $CAMERA_DEVICE"
    echo "可用设备:"
    ls -l /dev/video* 2>/dev/null || echo "  没有找到摄像头设备"
    exit 1
fi

echo "🚀 启动挖掘机程序..."
echo "📡 信令服务器: $SIGNALING_SERVER"
echo "📹 摄像头: $CAMERA_DEVICE (${VIDEO_WIDTH}x${VIDEO_HEIGHT}@${VIDEO_FPS}fps)"
echo "💡 按 Ctrl+C 停止"
echo ""

./excavator \
    -signaling "$SIGNALING_SERVER" \
    -camera "$CAMERA_DEVICE" \
    -width $VIDEO_WIDTH \
    -height $VIDEO_HEIGHT \
    -fps $VIDEO_FPS

