#!/usr/bin/env python3
"""
ROS2 Chassis Feedback -> stdout (JSON lines)
精简版：只保留驾驶核心数据，去除冗余诊断信息
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import time
import argparse


class TelemetryBridge(Node):
    def __init__(self, topic: str, device_id: str, rate_hz: float):
        super().__init__('telemetry_bridge')
        
        self.device_id = device_id
        self.rate_hz = rate_hz
        self.seq = 0
        self.latest_data = None
        self.min_interval = 1.0 / rate_hz
        
        self.subscription = self.create_subscription(
            String,
            topic,
            self.on_message,
            10
        )
        
        self.timer = self.create_timer(self.min_interval, self.send_telemetry)
        
        self.get_logger().info(f'Telemetry Bridge (Lite) Started. Topic: {topic}')

    def on_message(self, msg: String):
        try:
            self.latest_data = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def send_telemetry(self):
        if self.latest_data is None:
            return
        
        raw = self.latest_data
        self.seq += 1
        
        # ✂️ 精简后的数据结构 - 只保留驾驶员仪表盘需要的
        telemetry = {
            "id": self.device_id,
            "ts": int(time.time() * 1000),
            "seq": self.seq,
            
            # 基础状态
            "mode": raw.get("vehicle_mode", 0),   # 0:待机 1:遥控 ...
            "fault": raw.get("fault_level", 0),   # 0:正常 >0:故障码
            "batt": raw.get("battery_level", 0),  # 电池百分比
            
            # 关键指示灯 (UI 上显示图标)
            "flags": {
                "estop": raw.get("emergency_stop", False),   # 急停 (最重要!)
                "park": raw.get("parking_brake", False),     # 手刹
                "lock": raw.get("hydraulic_lock", False),    # 液压锁
                "horn": raw.get("horn_status", False),       # 喇叭
                "light": raw.get("work_light", False) or raw.get("low_beam", False),  # 灯光总状态
            },
            
            # 运动反馈 (仪表盘指针/数字)
            "drive": {
                "gear": raw.get("gear_signal", 0),           # 档位
                "spd_mode": "R" if raw.get("turtle_rabbit_gear", False) else "T",  # 兔子/乌龟
                "speed": raw.get("vehicle_speed", 0),        # 车速
                "steer": raw.get("front_rear_angle", 0.0),   # 铰接转向角度
                "throt": raw.get("throttle_opening", 0),     # 油门反馈 (0-100)
                "brake": raw.get("brake_opening", 0),        # 刹车反馈
            },
            
            # 姿态可视化 (用于 3D 模型同步显示)
            "pose": {
                "boom": raw.get("boom_angle", 0.0),          # 大臂角度
                "bucket": raw.get("bucket_angle", 0.0),      # 铲斗角度
            }
        }
        
        # 诊断数据(电流、电压、具体阀故障)全部删除！
        # 那些是给维修后台看的，不是给司机看的。
        
        try:
            json_line = json.dumps(telemetry, separators=(',', ':'))
            sys.stdout.write(json_line + '\n')
            sys.stdout.flush()
        except BrokenPipeError:
            self.get_logger().error('stdout 管道已断开，退出')
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='ROS2 Telemetry Bridge (Lite)')
    parser.add_argument('--topic', default='/cannode/chassis_feedback', help='ROS2 话题')
    parser.add_argument('--device-id', default='excavator_xc958', help='设备 ID')
    parser.add_argument('--rate', type=float, default=20.0, help='发送频率 (Hz)')
    args = parser.parse_args()
    
    rclpy.init()
    node = TelemetryBridge(args.topic, args.device_id, args.rate)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
