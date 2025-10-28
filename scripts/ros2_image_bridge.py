#!/usr/bin/env python3
"""
ROS2 图像桥接脚本
将 sensor_msgs/Image 话题转换为 GStreamer 可用的 UDP JPEG 流
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import sys

class ImageBridge(Node):
    def __init__(self, topic='/camera_front_wide', udp_port=5000, quality=80):
        super().__init__('image_bridge')
        
        self.bridge = CvBridge()
        self.udp_port = udp_port
        self.quality = quality
        self.frame_count = 0
        
        # 创建 UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 订阅图像话题
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10)
        
        self.get_logger().info(f'✅ 图像桥接已启动')
        self.get_logger().info(f'📡 订阅话题: {topic}')
        self.get_logger().info(f'📤 UDP 端口: {udp_port}')
        self.get_logger().info(f'🎨 JPEG 质量: {quality}')
        
    def image_callback(self, msg):
        try:
            # 转换为 OpenCV 图像
            if msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif msg.encoding == 'bgr8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif msg.encoding == 'mono8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 编码为 JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            _, jpeg_data = cv2.imencode('.jpg', cv_image, encode_param)
            
            # 通过 UDP 发送
            self.sock.sendto(jpeg_data.tobytes(), ('127.0.0.1', self.udp_port))
            
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # 每30帧打印一次
                self.get_logger().info(f'📹 已发送 {self.frame_count} 帧 ({len(jpeg_data)} 字节)')
                
        except Exception as e:
            self.get_logger().error(f'❌ 转换失败: {e}')
    
    def destroy_node(self):
        self.sock.close()
        super().destroy_node()

def main():
    # 解析命令行参数
    topic = sys.argv[1] if len(sys.argv) > 1 else '/camera_front_wide'
    udp_port = int(sys.argv[2]) if len(sys.argv) > 2 else 5000
    quality = int(sys.argv[3]) if len(sys.argv) > 3 else 80
    
    print("🌉 ROS2 图像桥接")
    print(f"📡 话题: {topic}")
    print(f"📤 UDP: 127.0.0.1:{udp_port}")
    print(f"🎨 质量: {quality}")
    print("")
    
    rclpy.init()
    
    try:
        bridge = ImageBridge(topic, udp_port, quality)
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\n🛑 停止桥接...")
    finally:
        if 'bridge' in locals():
            bridge.destroy_node()
        rclpy.shutdown()
        print("✅ 已退出")

if __name__ == '__main__':
    main()

