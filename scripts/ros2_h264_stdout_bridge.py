#!/usr/bin/env python3

"""
ROS2 H.264 <=> Stdout/Stdin 双向桥接

- [视频] 订阅 sensor_msgs/CompressedImage (h264)
- [视频] 将 H.264 码流 + 8字节时间戳写入标准输出 (stdout)
- [控制] 从标准输入 (stdin) 读取控制指令 JSON
- [控制] 将指令发布到 std_msgs/msg/String 话题
- 供父进程（如 Go 程序）高效交互
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String as StringMsg
import sys
import argparse
import threading
import json
import struct

class BidirectionalBridge(Node):
    def __init__(self, video_topic: str, control_topic: str):
        super().__init__('bidirectional_stdout_stdin_bridge')

        # 1. 视频订阅 (ROS2 -> Go)
        self.subscription = self.create_subscription(
            CompressedImage,
            video_topic,
            self.image_callback,
            10  # QoS profile depth
        )

        # 2. 控制指令发布 (Go -> ROS2)
        self.publisher = self.create_publisher(
            StringMsg,
            control_topic,
            10
        )

        self.get_logger().info(f'✅ 双向桥接启动')
        self.get_logger().info(f'  -> 视频订阅: {video_topic}')
        self.get_logger().info(f'  <- 控制发布: {control_topic}')
        self.get_logger().info('  -> H.264 + 时戳 -> stdout')
        self.get_logger().info('  <- 控制指令 JSON <- stdin')

        # 3. 启动一个独立线程来监听来自 Go 的控制指令
        self.control_thread = threading.Thread(target=self.listen_for_control_commands)
        self.control_thread.daemon = True
        self.control_thread.start()

    def image_callback(self, msg: CompressedImage):
        if msg.format != 'h264':
            self.get_logger().warn(f'收到非 h264 格式图像 ({msg.format})，已忽略')
            return

        try:
            h264_data = msg.data.tobytes()
            
            # 协议: [4字节长度][8字节纳秒时间戳][H.264数据]
            # 1. 计算纳秒时间戳
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            
            # 2. 打包为二进制
            header = struct.pack('>LQ', len(h264_data), timestamp_ns)

            # 3. 写入标准输出
            sys.stdout.buffer.write(header)
            sys.stdout.buffer.write(h264_data)
            sys.stdout.flush()

        except Exception as e:
            self.get_logger().error(f'处理视频帧时出错: {e}')
            sys.exit(1)

    def listen_for_control_commands(self):
        """
        在独立线程中运行，从 stdin 读取控制指令并发布到 ROS2
        """
        self.get_logger().info('控制指令监听线程已启动...')
        for line in sys.stdin:
            try:
                # Go 程序会发送带换行符的 JSON
                line = line.strip()
                if not line:
                    continue
                
                # 我们不需要解析 JSON，直接透传
                ros_msg = StringMsg()
                ros_msg.data = line
                self.publisher.publish(ros_msg)
                # self.get_logger().info(f'已发布控制指令: {line[:50]}...') # 调试用

            except json.JSONDecodeError:
                self.get_logger().warn(f'收到无效的 JSON 控制指令: {line}')
            except Exception as e:
                self.get_logger().error(f'处理控制指令时出错: {e}')


def main():
    parser = argparse.ArgumentParser(description='ROS2 H.264/Control Bidirectional Bridge')
    parser.add_argument('--video-topic', default='/camera_front_wide', help='要订阅的 CompressedImage 话题')
    parser.add_argument('--control-topic', default='/controls/teleop', help='要发布的控制指令话题')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    
    bridge_node = None
    try:
        bridge_node = BidirectionalBridge(
            video_topic=args.video_topic,
            control_topic=args.control_topic
        )
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if bridge_node:
            bridge_node.get_logger().fatal(f'桥接意外终止: {e}')
        else:
            print(f'桥接意外终止: {e}', file=sys.stderr)
    finally:
        if bridge_node:
            bridge_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
