#!/bin/bash
#
# 启动 Isaac Sim H.264 发布器节点的脚本
#

# --- 可配置变量 ---
# Isaac Sim 发布的原始图像话题
INPUT_TOPIC="/front_camera"
# 编码后要发布到的话题 (必须与 ros2_h264_stdout_bridge.py 的输入匹配)
OUTPUT_TOPIC="/camera/image_h264"
WIDTH=1280
HEIGHT=720
FPS=30
BITRATE=8000000 # 8 Mbps, for high quality streams

# --- 脚本主体 ---
set -e

# 获取脚本所在目录的绝对路径
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")
LOG_DIR="$PROJECT_ROOT/logs"

# 检查并创建日志目录
if [ ! -d "$LOG_DIR" ]; then
    mkdir -p "$LOG_DIR"
fi

# 检查日志目录的写入权限
if [ ! -w "$LOG_DIR" ]; then
    echo "错误：对日志目录 ($LOG_DIR) 没有写入权限。"
    echo "请尝试以管理员身份执行 'sudo chown -R $(whoami) $PROJECT_ROOT' 来修复整个项目文件夹的权限。"
    exit 1
fi

# 日志文件名
LOG_FILE="$LOG_DIR/ros2-isaac-h264-publisher.log"

# 检查 ROS2 环境是否已 source
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS2 环境未加载. 请先执行 'source /opt/ros/humble/setup.bash'"
    exit 1
fi

echo "--- 启动 Isaac Sim H.264 Publisher ---"
echo "原始图像话题 (输入): $INPUT_TOPIC"
echo "H.264 话题 (输出) : $OUTPUT_TOPIC"
echo "分辨率           : ${WIDTH}x${HEIGHT}@${FPS}fps"
echo "码率             : $BITRATE bps"
echo "日志文件         : $LOG_FILE"
echo "------------------------------------"

# 定义清理函数
cleanup() {
    echo " "
    echo "--- 收到退出信号, 正在清理 Isaac Publisher ---"
    # 使用 pkill 和 -f 标志来杀死包含特定名称的进程
    pkill -f "ros2_isaac_h264_publisher.py"
    echo "Isaac Publisher 进程已停止."
    exit 0
}

# 捕获退出信号
trap cleanup SIGINT SIGTERM

# 在后台运行 Python 节点，并将日志输出到文件
# 使用 stdbuf -o0 来禁用输出缓冲，确保日志实时写入文件
stdbuf -o0 python3 "$SCRIPT_DIR/ros2_isaac_h264_publisher.py" \
    --input-topic "$INPUT_TOPIC" \
    --output-topic "$OUTPUT_TOPIC" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --fps "$FPS" \
    --bitrate "$BITRATE" \
    --ros-args -r __node:=isaac_h264_publisher \
    > "$LOG_FILE" 2>&1 &

# 等待后台进程，这样 trap 才能生效
wait $!
