#!/bin/bash

# ROS2 功能测试脚本

set -e

echo "🧪 ROS2 集成测试"
echo ""

# 检查 ROS2 是否可用
if ! command -v ros2 >/dev/null 2>&1; then
    echo "❌ 未找到 ros2 命令"
    echo "请安装 ROS2 或 source 环境："
    echo "  source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS2 环境已就绪"
echo ""

# 显示 ROS2 信息
echo "📋 ROS2 信息:"
echo "   版本: $(ros2 --version 2>&1 | head -1)"
echo "   域 ID: ${ROS_DOMAIN_ID:-0}"
echo ""

# 列出当前话题
echo "📡 当前 ROS2 话题:"
ros2 topic list | head -10 || echo "  (无话题)"
echo ""

# 测试控制话题
CONTROL_TOPIC="/controls/teleop"
echo "1️⃣ 测试控制话题: $CONTROL_TOPIC"
echo ""
echo "   启动监听（新终端中运行）:"
echo "   ros2 topic echo $CONTROL_TOPIC std_msgs/msg/String"
echo ""

read -p "是否发送测试控制消息？(y/n): " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "   📤 发送测试消息..."
    
    TEST_MSG='{
      "rotation": 0.5,
      "throttle": 0.8,
      "brake": 0.0,
      "gear": "D",
      "boom": 0.3,
      "bucket": -0.2,
      "device_type": "wheel_loader",
      "timestamp": '$(date +%s000)'
    }'
    
    # 移除换行和多余空格
    TEST_MSG_COMPACT=$(echo "$TEST_MSG" | tr -d '\n' | tr -s ' ')
    
    ros2 topic pub --once "$CONTROL_TOPIC" std_msgs/msg/String \
        "{data: '$TEST_MSG_COMPACT'}"
    
    echo "   ✅ 测试消息已发送"
fi

echo ""

# 测试图像话题
IMAGE_TOPIC="/camera_front_wide"
echo "2️⃣ 检查图像话题: $IMAGE_TOPIC"

if ros2 topic list | grep -q "$IMAGE_TOPIC"; then
    echo "   ✅ 话题存在"
    echo ""
    echo "   话题信息:"
    ros2 topic info "$IMAGE_TOPIC" || true
    echo ""
    echo "   发布频率:"
    timeout 3s ros2 topic hz "$IMAGE_TOPIC" 2>/dev/null || echo "   (无数据或超时)"
else
    echo "   ⚠️  话题不存在"
    echo "   如需使用 ROS2 视频源，请先启动摄像头节点"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ 测试完成！"
echo ""
echo "💡 下一步:"
echo "   1. 启动挖掘机程序并启用 ROS2:"
echo "      ./bin/excavator -signaling ws://... -enable-ros2 true"
echo ""
echo "   2. 在浏览器控制页面发送指令"
echo ""
echo "   3. 监听控制话题:"
echo "      ros2 topic echo /controls/teleop std_msgs/msg/String"
echo ""

