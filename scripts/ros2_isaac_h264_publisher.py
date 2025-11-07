#!/usr/bin/env python3

"""
ROS2 Isaac Sim Bridge Node (x86 Hardware Encoding H.264)
- Subscribes to a raw sensor_msgs/Image topic from Isaac Sim.
- Uses GStreamer with `appsrc` to inject frames.
- Encodes the stream to H.264 using NVIDIA hardware (`nvh264enc`).
- Publishes the result to a sensor_msgs/CompressedImage topic.

Dependencies:
- rclpy, sensor_msgs, cv_bridge
- PyGObject (GStreamer): python3-gi, gir1.2-gst-1.0
- GStreamer plugins (good, bad, ugly) including nvh264enc.
- OpenCV for Python (cv2)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import cv2
from cv_bridge import CvBridge
import numpy as np

class IsaacH264Publisher(Node):
    def __init__(self,
                 input_topic: str = '/front_camera',
                 output_topic: str = '/camera/image_h264',
                 width: int = 1280,
                 height: int = 720,
                 fps: int = 30,
                 bitrate: int = 4000000):
        super().__init__('isaac_h264_publisher')

        self.input_topic = input_topic
        self.output_topic = output_topic
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate_kbps = int(bitrate / 1000) # nvh264enc expects bitrate in kbps
        self.frame_duration_ns = int(1e9 // max(1, self.fps))
        self.timestamp_ns = 0
        
        self.publisher_ = self.create_publisher(CompressedImage, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10)
        
        self.bridge = CvBridge()
        self.get_logger().info(f'Subscribing to raw image topic (RGB expected): {self.input_topic}')

        Gst.init(None)

        # GStreamer pipeline for x86 with NVIDIA GPU
        # appsrc -> videoconvert -> correct format -> nvh264enc -> h264parse -> appsink
        pipeline_str = (
            f"appsrc name=appsrc is-live=true format=GST_FORMAT_time do-timestamp=true ! "
            f"video/x-raw,format=RGB,width={self.width},height={self.height},framerate={self.fps}/1 ! "
            "videoconvert ! "
            "video/x-raw,format=I420 ! "
            f"nvh264enc bitrate={self.bitrate_kbps} preset=low-latency-hq tune=zerolatency key-int-max={self.fps} insert-sps-pps=true ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! "
            "h264parse config-interval=-1 ! "
            "appsink name=sink emit-signals=true max-buffers=5 drop=true sync=false"
        )

        self.get_logger().info(f'GStreamer Pipeline: {pipeline_str}')

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name('appsrc')
        # Set caps explicitly to ensure correct negotiation
        caps = Gst.Caps.from_string(
            f"video/x-raw,format=RGB,width={self.width},height={self.height},framerate={self.fps}/1"
        )
        self.appsrc.set_property('caps', caps)
        self.appsrc.set_property('is-live', True)
        self.appsrc.set_property('block', True)
        self.appsrc.set_property('format', Gst.Format.TIME)
        self.appsrc.set_property('do-timestamp', True)

        self.sink = self.pipeline.get_by_name('sink')
        self.sink.connect('new-sample', self.on_new_sample)

        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self.on_gst_error)
        bus.connect("message::eos", self.on_gst_eos)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('Failed to start GStreamer pipeline.')

        self.get_logger().info(f'Node started: Publishing H.264 from {self.input_topic} to {self.output_topic}')

    def _ensure_size_rgb(self, image_rgb: np.ndarray) -> np.ndarray:
        if image_rgb.shape[1] != self.width or image_rgb.shape[0] != self.height:
            resized = cv2.resize(image_rgb, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
            return resized
        return image_rgb

    def image_callback(self, msg: Image):
        """Callback for new raw images from Isaac Sim."""
        try:
            # Convert ROS Image message to an OpenCV image (numpy array)
            # Isaac Sim often publishes in RGB8 format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion error: {e}')
            return

        # --- Fix 3: Ensure Frame Size Consistency ---
        # If the incoming frame size does not match the pipeline's configured size,
        # resize it to prevent the GStreamer pipeline from stalling or erroring.
        if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
            self.get_logger().warn(
                f'Incoming image size ({cv_image.shape[1]}x{cv_image.shape[0]}) '
                f'does not match pipeline config ({self.width}x{self.height}). Resizing...'
            )
            cv_image = cv2.resize(cv_image, (self.width, self.height), interpolation=cv2.INTER_LINEAR)

        # --- Fix 1: Zero-Copy Buffer Wrapping ---
        # Ensure the numpy array is contiguous in memory.
        # This is a prerequisite for Gst.Buffer.new_wrapped.
        frame = np.ascontiguousarray(cv_image)

        # Directly wrap the numpy array's underlying memory without copying.
        buf = Gst.Buffer.new_wrapped(frame.data)

        # --- Fix 2: Use Authoritative Timestamps ---
        # Use the real timestamp from the ROS message header.
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        
        buf.pts = timestamp_ns
        buf.dts = timestamp_ns # For simple streams, DTS can be the same as PTS
        buf.duration = self.frame_duration_ns

        # Push the buffer into the GStreamer pipeline
        ret = self.appsrc.emit('push-buffer', buf)
        if ret != Gst.FlowReturn.OK:
            self.get_logger().warning(f"GStreamer appsrc push-buffer returned {ret}")

    def on_new_sample(self, sink):
        """Callback for new H.264 samples from the GStreamer pipeline."""
        sample = sink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        try:
            data = bytes(mapinfo.data)
        finally:
            buf.unmap(mapinfo)
        
        # Publish H.264 CompressedImage
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'h264'
        msg.data = data
        self.publisher_.publish(msg)

        return Gst.FlowReturn.OK

    def shutdown(self):
        """Cleanly shuts down the GStreamer pipeline and ROS node."""
        self.get_logger().info('Shutting down...')
        if hasattr(self, 'pipeline') and self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
            self.get_logger().info('GStreamer pipeline set to NULL')

    def on_gst_error(self, bus, message):
        """GStreamer error callback."""
        err, debug = message.parse_error()
        self.get_logger().error(f"GStreamer error: {err}, {debug}")
        self.shutdown()

    def on_gst_eos(self, bus, message):
        """GStreamer EOS (End-of-Stream) callback."""
        self.get_logger().info("GStreamer End-of-Stream.")
        self.shutdown()


def main():
    import argparse
    parser = argparse.ArgumentParser(description='ROS2 Isaac Sim H.264 Publisher (NVIDIA HW Encode)')
    parser.add_argument('--input-topic', default='/front_camera', help='ROS2 topic for raw sensor_msgs/Image')
    parser.add_argument('--output-topic', default='/camera/image_h264', help='ROS2 topic for compressed H.264 output')
    parser.add_argument('--width', type=int, default=1280, help='Image width')
    parser.add_argument('--height', type=int, default=720, help='Image height')
    parser.add_argument('--fps', type=int, default=30, help='Frames per second')
    parser.add_argument('--bitrate', type=int, default=4000000, help='Target bitrate in bps (e.g., 4000000)')
    args, unknown = parser.parse_known_args() # Use parse_known_args to ignore ROS arguments

    rclpy.init()
    try:
        node = IsaacH264Publisher(
            input_topic=args.input_topic,
            output_topic=args.output_topic,
            width=args.width,
            height=args.height,
            fps=args.fps,
            bitrate=args.bitrate,
        )

        rclpy.spin(node)

    except KeyboardInterrupt:
        if 'node' in locals():
            node.get_logger().info('Ctrl+C detected.')
    except Exception as e:
        # Cannot use rclpy logger if node initialization failed
        print(f'Node failed to start: {e}')
    finally:
        if 'node' in locals() and rclpy.ok():
            node.shutdown()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
