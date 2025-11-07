#!/usr/bin/env python3
"""
Shared Memory to H.264 Publisher (Pure GStreamer + ROS2 Publisher - NO Isaac Sim deps)

This script reads raw RGB data from shared memory, encodes it using
NVIDIA hardware acceleration (nvh264enc), and publishes the H.264 stream
to a ROS2 topic.

Author: AI Assistant
Date: 2025-11-05
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import struct
import mmap
import os
import time


class ShmToH264Publisher(Node):
    """Publisher that reads from shared memory and publishes H.264."""

    def __init__(self):
        super().__init__('shm_to_h264_publisher')

        # Declare parameters
        self.declare_parameter('output_topic', '/camera/image_h264')
        self.declare_parameter('shm_path', '/dev/shm/isaac_rgb_buffer')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 20)
        self.declare_parameter('bitrate_kbps', 4000)
        
        self.output_topic = self.get_parameter('output_topic').value
        self.shm_path = self.get_parameter('shm_path').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.bitrate_kbps = self.get_parameter('bitrate_kbps').value
        
        # Calculate buffer sizes
        self.header_size = 32
        self.image_size = self.width * self.height * 3
        self.total_size = self.header_size + self.image_size
        
        # Wait for shared memory file to be created
        max_wait = 10  # seconds
        wait_start = time.time()
        while not os.path.exists(self.shm_path):
            if time.time() - wait_start > max_wait:
                raise RuntimeError(f'Shared memory file not found: {self.shm_path}')
            self.get_logger().info(f'Waiting for shared memory file: {self.shm_path}')
            time.sleep(0.5)
        
        # Open shared memory for reading
        try:
            self.shm_fd = open(self.shm_path, 'rb')
            self.shm_mmap = mmap.mmap(self.shm_fd.fileno(), self.total_size, access=mmap.ACCESS_READ)
            self.get_logger().info(f'Opened shared memory at {self.shm_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to open shared memory: {e}')
            raise
        
        # QoS profile for video streaming
        video_qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Create ROS2 publisher
        self.publisher = self.create_publisher(
            CompressedImage,
            self.output_topic,
            video_qos_profile
        )
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Build GStreamer pipeline with hardware encoding
        # We use appsrc to push data from shared memory
        pipeline_str = (
            f"appsrc name=appsrc is-live=true format=time do-timestamp=false block=true ! "
            f"video/x-raw,format=RGB,width={self.width},height={self.height},framerate={self.fps}/1 ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420 ! "
            f"nvh264enc bitrate={self.bitrate_kbps} preset=low-latency-hq rc-mode=cbr-ld-hq "
            f"zerolatency=true gop-size={self.fps} ! "
            f"video/x-h264,stream-format=byte-stream,alignment=au,profile=baseline ! "
            f"h264parse config-interval=-1 ! "
            f"appsink name=sink emit-signals=true max-buffers=5 drop=true sync=false"
        )
        
        self.get_logger().info(f'GStreamer Pipeline: {pipeline_str}')
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
        except Exception as e:
            self.get_logger().error(f'Failed to create GStreamer pipeline: {e}')
            raise
        
        # Get appsrc element
        self.appsrc = self.pipeline.get_by_name('appsrc')
        if not self.appsrc:
            raise RuntimeError('Failed to get appsrc element')
        
        # Configure appsrc
        self.appsrc.set_property('format', Gst.Format.TIME)
        self.appsrc.set_property('is-live', True)
        self.appsrc.set_property('do-timestamp', False)
        self.appsrc.set_property('block', True)
        
        # Get appsink element
        self.sink = self.pipeline.get_by_name('sink')
        if not self.sink:
            raise RuntimeError('Failed to get appsink element')
        
        self.sink.connect('new-sample', self.on_new_sample)
        
        # Connect bus signals
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::error", self.on_gst_error)
        bus.connect("message::eos", self.on_gst_eos)
        bus.connect("message::warning", self.on_gst_warning)
        
        # Start GStreamer pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError('Failed to start GStreamer pipeline')
        
        self.get_logger().info('GStreamer pipeline started')
        
        # Frame tracking
        self.last_frame_counter = -1
        self.frames_pushed = 0
        self.frames_published = 0
        
        # Statistics
        self.last_stats_time = time.time()
        self.frames_since_last_stats = 0
        
        # Create a timer to read from shared memory and push to GStreamer
        # We use a timer instead of blocking to keep ROS2 spinning
        self.timer = self.create_timer(1.0 / (self.fps * 2), self.timer_callback)
        
        self.get_logger().info(f'Publishing H.264 to: {self.output_topic}')
        self.get_logger().info('Node started successfully')

    def timer_callback(self):
        """Timer callback to read from shared memory and push to GStreamer."""
        try:
            # Read header from shared memory
            self.shm_mmap.seek(0)
            header_data = self.shm_mmap.read(self.header_size)
            
            if len(header_data) < self.header_size:
                return
            
            # Parse header
            timestamp_ns, width, height, frame_counter = struct.unpack('QIIQ8x', header_data)
            
            # Check if this is a new frame
            if frame_counter == self.last_frame_counter:
                return  # Same frame, skip
            
            # Validate dimensions
            if width != self.width or height != self.height:
                self.get_logger().error(
                    f'Dimension mismatch in SHM: {width}x{height}, expected {self.width}x{self.height}',
                    throttle_duration_sec=5.0
                )
                return
            
            # Read image data
            image_data = self.shm_mmap.read(self.image_size)
            
            if len(image_data) != self.image_size:
                self.get_logger().error(
                    f'Incomplete image data: {len(image_data)}, expected {self.image_size}',
                    throttle_duration_sec=5.0
                )
                return
            
            # Create GStreamer buffer (no extra copy)
            buf = Gst.Buffer.new_wrapped(image_data)
            
            # Set timestamps
            buf.pts = timestamp_ns
            buf.dts = timestamp_ns
            buf.duration = Gst.SECOND // self.fps
            
            # Push buffer to appsrc
            ret = self.appsrc.emit('push-buffer', buf)
            if ret != Gst.FlowReturn.OK:
                self.get_logger().warning(f'appsrc push-buffer returned {ret}', throttle_duration_sec=2.0)
            else:
                self.frames_pushed += 1
                self.last_frame_counter = frame_counter
            
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {e}', throttle_duration_sec=2.0)

    def on_new_sample(self, sink):
        """Callback when new encoded H.264 sample is available."""
        try:
            sample = sink.emit('pull-sample')
            if not sample:
                return Gst.FlowReturn.ERROR
            
            buf = sample.get_buffer()
            if not buf:
                return Gst.FlowReturn.ERROR
            
            # Extract H.264 data
            success, map_info = buf.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.ERROR
            
            h264_data = bytes(map_info.data)
            buf.unmap(map_info)
            
            # Publish to ROS2
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'h264'
            msg.data = h264_data
            
            self.publisher.publish(msg)
            self.frames_published += 1
            self.frames_since_last_stats += 1
            
            # Print statistics
            current_time = time.time()
            if current_time - self.last_stats_time >= 2.0:
                fps = self.frames_since_last_stats / (current_time - self.last_stats_time)
                self.get_logger().info(
                    f'Publishing H.264: {fps:.1f} fps, '
                    f'Pushed: {self.frames_pushed}, Published: {self.frames_published}'
                )
                self.last_stats_time = current_time
                self.frames_since_last_stats = 0
            
            return Gst.FlowReturn.OK
            
        except Exception as e:
            self.get_logger().error(f'Error in on_new_sample: {e}')
            return Gst.FlowReturn.ERROR

    def on_gst_error(self, bus, message):
        """Handle GStreamer error messages."""
        err, debug = message.parse_error()
        self.get_logger().error(f'GStreamer Error: {err}, Debug: {debug}')
        self.shutdown()

    def on_gst_eos(self, bus, message):
        """Handle GStreamer EOS messages."""
        self.get_logger().info('GStreamer EOS received')
        self.shutdown()

    def on_gst_warning(self, bus, message):
        """Handle GStreamer warning messages."""
        warn, debug = message.parse_warning()
        self.get_logger().warning(f'GStreamer Warning: {warn}, Debug: {debug}')

    def shutdown(self):
        """Clean shutdown of GStreamer and shared memory."""
        try:
            if hasattr(self, 'pipeline'):
                self.pipeline.set_state(Gst.State.NULL)
            if hasattr(self, 'shm_mmap'):
                self.shm_mmap.close()
            if hasattr(self, 'shm_fd'):
                self.shm_fd.close()
            self.get_logger().info('Shutdown complete')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ShmToH264Publisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

