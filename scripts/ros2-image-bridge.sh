#!/bin/bash

# ROS2 图像桥接脚本
# 将 ROS2 sensor_msgs/Image 话题转换为 GStreamer 可用的视频流

set -e

ROS2_TOPIC="${1:-/camera_front_wide}"
OUTPUT_PORT="${2:-5000}"

echo "🌉 启动 ROS2 图像桥接..."
echo "📡 订阅话题: $ROS2_TOPIC"
echo "📤 输出端口: $OUTPUT_PORT"
echo ""

# 检查话题是否存在
if ! ros2 topic list | grep -q "^$ROS2_TOPIC$"; then
    echo "❌ 话题不存在: $ROS2_TOPIC"
    echo "可用话题:"
    ros2 topic list | grep camera || echo "  (无摄像头话题)"
    exit 1
fi

# 获取话题信息
echo "📋 话题信息:"
ros2 topic info "$ROS2_TOPIC"
echo ""

# 方案 1: 使用 image_view (需要安装 image_pipeline)
if command -v ros2 run image_view video_saver >/dev/null 2>&1; then
    echo "✅ 使用 image_view 转换"
    echo "⚠️  此方案需要 X11 显示"
    
    # 使用 image_view 将图像保存为视频文件
    # ros2 run image_view video_saver image:=$ROS2_TOPIC _filename:=/tmp/ros2_video.avi
    
    echo "💡 建议使用 GStreamer 插件方案（见下方）"
fi

# 方案 2: 使用 gst-launch 和共享内存（推荐）
echo ""
echo "🔧 方案 2: 使用 GStreamer rtpvrawpay/depay"
echo ""
echo "步骤:"
echo "1. 在一个终端运行 ROS2 图像发布者（已运行）"
echo ""
echo "2. 运行此桥接脚本（使用 Python）:"
cat << 'PYTHON_SCRIPT'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket

class ImageBridge(Node):
    def __init__(self):
        super().__init__('image_bridge')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera_front_wide',
            self.image_callback,
            10)
        
        # 创建 UDP socket 发送到 GStreamer
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.get_logger().info('图像桥接已启动')

    def image_callback(self, msg):
        try:
            # 转换为 OpenCV 图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 编码为 JPEG
            _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # 通过 UDP 发送（可以被 GStreamer udpsrc 接收）
            self.sock.sendto(jpeg.tobytes(), ('127.0.0.1', 5000))
            
        except Exception as e:
            self.get_logger().error(f'转换失败: {e}')

def main():
    rclpy.init()
    bridge = ImageBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
PYTHON_SCRIPT

echo ""
echo "3. 将上述代码保存为 ros2_image_bridge.py 并运行"
echo ""

# 方案 3: 直接使用 GStreamer ROS2 插件（如果可用）
echo "🔧 方案 3: 使用 gst-launch (需要 ros_gst_bridge)"
echo ""
echo "gst-launch-1.0 ros2src topic=$ROS2_TOPIC ! videoconvert ! udpsink host=127.0.0.1 port=5000"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "💡 推荐方案：使用 Python 桥接脚本"
echo ""
echo "创建桥接脚本:"
echo "  nano ~/ros2_image_bridge.py"
echo "  # 复制上面的 Python 代码"
echo ""
echo "运行桥接:"
echo "  python3 ~/ros2_image_bridge.py"
echo ""
echo "然后修改 excavator 使用 udpsrc:"
echo "  修改 createROS2VideoSource 返回:"
echo "  udpsrc port=5000 ! jpegdec ! videoconvert ! video/x-raw,format=I420"
echo ""

