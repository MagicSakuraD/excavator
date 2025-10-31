#!/usr/bin/env python3

"""
ROS2 摄像头发布节点（Jetson/Orin 硬件编码 H.264）
- 从 /dev/videoX 读取
- 使用 GStreamer nvv4l2h264enc 硬件编码为 H.264（Annex-B）
- 发布到 sensor_msgs/CompressedImage，format="h264"

依赖：
- rclpy, sensor_msgs
- PyGObject (GStreamer): python3-gi, gir1.2-gst-1.0
- GStreamer 插件（含 nvv4l2h264enc）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class H264CameraPublisher(Node):
    def __init__(self,
                 device: str = '/dev/video0',
                 width: int = 640,
                 height: int = 480,
                 fps: int = 30,
                 bitrate: int = 4000000,
                 topic: str = '/camera_front_wide'):
        super().__init__('h264_camera_publisher')

        self.topic = topic
        self.publisher_ = self.create_publisher(CompressedImage, topic, 10)

        Gst.init(None)

        # 关键：Annex-B、AU 对齐、周期性 SPS/PPS
        pipeline_str = (
            f"v4l2src device={device} ! "
            f"video/x-raw,width={width},height={height},framerate={fps}/1 ! "
            "nvvidconv ! "
            "video/x-raw(memory:NVMM),format=NV12 ! "
            f"nvv4l2h264enc bitrate={bitrate} preset-level=1 insert-sps-pps=true idrinterval={fps} iframeinterval={fps} ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! "
            "h264parse config-interval=-1 ! "
            "appsink name=sink emit-signals=true max-buffers=4 drop=true sync=false"
        )

        self.get_logger().info(f'🎬 GStreamer: {pipeline_str}')

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.sink = self.pipeline.get_by_name('sink')
        self.sink.connect('new-sample', self.on_new_sample)

        # 监控 GStreamer 总线消息 (错误/EOS)
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self.on_gst_error)
        bus.connect("message::eos", self.on_gst_eos)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('无法启动 GStreamer 管线')

        self.get_logger().info(f'✅ 启动完成: device={device}, {width}x{height}@{fps}, bitrate={bitrate}, topic={topic}')

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        size = buf.get_size()
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            data = bytes(mapinfo.data)
        finally:
            buf.unmap(mapinfo)

        # 发布 H.264 CompressedImage（Annex-B）
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'h264'
        msg.data = data
        self.publisher_.publish(msg)

        return Gst.FlowReturn.OK

    def shutdown(self):
        """安全关闭 GStreamer 和 ROS 节点"""
        self.get_logger().info('🔌 正在关闭...')
        if hasattr(self, 'pipeline') and self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.get_logger().info('...GStreamer 管线已设为 NULL')
        
    def on_gst_error(self, bus, message):
        """GStreamer 错误回调"""
        err, debug = message.parse_error()
        self.get_logger().error(f"❌ GStreamer 错误: {err}, {debug}")
        self.shutdown()
        # 可以在这里触发更激进的退出机制
        # import os
        # os._exit(1)

    def on_gst_eos(self, bus, message):
        """GStreamer EOS (End-of-Stream) 回调"""
        self.get_logger().info("🏁 GStreamer 到达流末尾 (EOS)")
        self.shutdown()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='ROS2 H.264 Camera Publisher (Jetson HW Encode)')
    parser.add_argument('--device', default='/dev/video0')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--bitrate', type=int, default=4000000)
    parser.add_argument('--topic', default='/camera_front_wide')
    args = parser.parse_args()

    rclpy.init()
    try:
        node = H264CameraPublisher(
            device=args.device,
            width=args.width,
            height=args.height,
            fps=args.fps,
            bitrate=args.bitrate,
            topic=args.topic,
        )

        loop = GLib.MainLoop()
        try:
            loop.run()
        except KeyboardInterrupt:
            node.get_logger().info('⌨️ 收到 Ctrl+C')
            pass
        finally:
            loop.quit()
            node.shutdown()
            node.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        # 使用 rclpy logger 记录启动错误
        # (此时节点可能还未完全初始化，所以用 print)
        print(f'💥 节点启动失败: {e}')

if __name__ == '__main__':
    main()



