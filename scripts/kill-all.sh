#!/bin/bash

# 停止所有相关进程

echo "🛑 停止所有 excavator 相关进程..."

# 1. 停止 Go/Python 进程
pkill -f "excavator" && echo "  ✅ 已停止 excavator 程序 (Go)" || echo "  ℹ️  excavator 程序未运行"
pkill -f "ros2_h264_stdout_bridge.py" && echo "  ✅ 已停止 H264 stdout 桥接 (Python)" || echo "  ℹ️  H264 stdout 桥接未运行"
pkill -f "ros2_h264_camera_publisher.py" && echo "  ✅ 已停止 H264 摄像头发布节点 (Python)" || echo "  ℹ️  H264 摄像头发布节点未运行"

# 2. (关键) 强制释放摄像头资源
# 查找并杀死所有占用 /dev/video0 的进程
CAM_DEVICE="/dev/video0"
PIDS_USING_CAM=$(fuser $CAM_DEVICE 2>/dev/null)

if [ -n "$PIDS_USING_CAM" ]; then
    echo "📹 发现有进程正在占用摄像头 $CAM_DEVICE (PIDs: $PIDS_USING_CAM)"
    # -k: kill, -9: SIGKILL (强制)
    fuser -k -9 $CAM_DEVICE 2>/dev/null
    echo "  ✅ 已强制释放摄像头资源"
else
    echo "  ℹ️  摄像头 $CAM_DEVICE 未被占用"
fi

echo "✅ 完成"

