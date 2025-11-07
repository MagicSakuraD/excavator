#!/usr/bin/env python3
"""
ROS2 Control Stdin Bridge (Pure ROS2)
- Reads lines from stdin (JSON/text)
- Publishes to a ROS2 topic (default: /controls/teleop)
- QoS: depth=1, BEST_EFFORT
"""

import sys
import threading
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String


class ControlStdinNode(Node):
    def __init__(self):
        super().__init__('control_stdin_bridge')
        self.declare_parameter('control_topic', '/controls/teleop')
        topic = self.get_parameter('control_topic').value

        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(String, topic, qos)
        self.get_logger().info(f'✅ ROS2 控制桥接已启动')
        self.get_logger().info(f'  发布到话题: {topic}')
        self.get_logger().info(f'  等待从 stdin 接收控制指令...')

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _reader_loop(self):
        """从 stdin 读取 JSON 控制指令并发布到 ROS2"""
        msg_count = 0
        for line in sys.stdin:
            if not line:
                break
            
            line = line.strip()
            if not line:
                continue
            
            # 验证 JSON 格式（可选，有助于调试）
            try:
                json.loads(line)  # 验证 JSON 格式
            except json.JSONDecodeError as e:
                self.get_logger().warn(f'收到无效的 JSON 格式: {e}')
                self.get_logger().debug(f'  内容: {line[:100]}...')
                continue
            
            # 发布到 ROS2 话题
            ros_msg = String()
            ros_msg.data = line
            self.pub.publish(ros_msg)
            
            msg_count += 1
            # 每 100 条消息记录一次日志（避免日志过多）
            if msg_count % 100 == 0:
                self.get_logger().info(f'已发布 {msg_count} 条控制指令')
        
        self.get_logger().info(f'stdin 已关闭，退出读取循环（共处理 {msg_count} 条消息）')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ControlStdinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
