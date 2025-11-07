#!/usr/bin/env python3
"""
ROS2 to Shared Memory Bridge (Pure ROS2 - NO GStreamer)

This script subscribes to Isaac Sim's raw RGB image topic and writes
the image data directly to shared memory for consumption by the
GStreamer encoding process.

Author: AI Assistant
Date: 2025-11-05
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
import mmap
import struct
import time
import os


class RosToShmBridge(Node):
    """Bridge that writes ROS2 Image messages to shared memory."""

    def __init__(self):
        super().__init__('ros_to_shm_bridge')

        # Declare parameters
        self.declare_parameter('input_topic', '/stitched_image')
        self.declare_parameter('shm_path', '/dev/shm/isaac_rgb_buffer')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        
        self.input_topic = self.get_parameter('input_topic').value
        self.shm_path = self.get_parameter('shm_path').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        
        # Calculate buffer size
        # Header: 32 bytes (timestamp 8 bytes + width 4 bytes + height 4 bytes + frame_counter 8 bytes + reserved 8 bytes)
        # Image data: width * height * 3 (RGB)
        self.header_size = 32
        self.image_size = self.width * self.height * 3
        self.total_size = self.header_size + self.image_size
        
        # Create shared memory file
        try:
            # Remove existing file if present
            if os.path.exists(self.shm_path):
                os.remove(self.shm_path)
                self.get_logger().info(f'Removed existing shared memory file: {self.shm_path}')
            
            # Create and initialize the shared memory file
            with open(self.shm_path, 'wb') as f:
                f.write(b'\x00' * self.total_size)
            
            # Open for memory mapping
            self.shm_fd = open(self.shm_path, 'r+b')
            self.shm_mmap = mmap.mmap(self.shm_fd.fileno(), self.total_size)
            
            self.get_logger().info(f'Created shared memory at {self.shm_path} ({self.total_size} bytes)')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create shared memory: {e}')
            raise
        
        # Frame counter for debugging
        self.frame_counter = 0
        
        # QoS profile for video streaming
        video_qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Subscribe to raw image topic
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            video_qos_profile
        )
        
        self.get_logger().info(f'Subscribing to: {self.input_topic}')
        self.get_logger().info(f'Expected format: rgb8, {self.width}x{self.height} (will adapt if different)')
        self.get_logger().info('Node started successfully')
        
        # Statistics
        self.last_stats_time = time.time()
        self.frames_since_last_stats = 0

    def _reallocate_shm(self, new_width: int, new_height: int):
        """Recreate the shared memory file/mapping for a new resolution."""
        try:
            # Close old mappings
            if hasattr(self, 'shm_mmap') and self.shm_mmap:
                self.shm_mmap.close()
            if hasattr(self, 'shm_fd') and self.shm_fd:
                self.shm_fd.close()

            # Update dimensions and sizes
            self.width = int(new_width)
            self.height = int(new_height)
            self.image_size = self.width * self.height * 3
            self.total_size = self.header_size + self.image_size

            # Recreate file with new size
            if os.path.exists(self.shm_path):
                os.remove(self.shm_path)
            with open(self.shm_path, 'wb') as f:
                f.write(b'\x00' * self.total_size)
            self.shm_fd = open(self.shm_path, 'r+b')
            self.shm_mmap = mmap.mmap(self.shm_fd.fileno(), self.total_size)

            self.get_logger().info(f'Reallocated SHM to {self.width}x{self.height} ({self.total_size} bytes)')
        except Exception as e:
            self.get_logger().error(f'Failed to reallocate SHM: {e}')

    def image_callback(self, msg: Image):
        """Callback for incoming image messages."""
        try:
            # Validate encoding
            if msg.encoding != 'rgb8':
                self.get_logger().error(
                    f'Unexpected encoding: {msg.encoding}, expected rgb8',
                    throttle_duration_sec=5.0
                )
                return
            
            # Adapt to dynamic resolution changes
            if msg.width != self.width or msg.height != self.height:
                self.get_logger().warn(
                    f'Resolution changed: {self.width}x{self.height} -> {msg.width}x{msg.height}. Reallocating SHM...',
                    throttle_duration_sec=1.0
                )
                self._reallocate_shm(msg.width, msg.height)
            
            # Validate data size
            expected_size = self.width * self.height * 3
            if len(msg.data) != expected_size:
                # 如果上游编码器在切换时出现短帧，直接跳过该帧，以免破坏消费者读取
                self.get_logger().warn(
                    f'Data size mismatch: got {len(msg.data)}, expect {expected_size}. Dropping frame.',
                    throttle_duration_sec=1.0
                )
                return
            
            # Get timestamp in nanoseconds
            timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            
            # Write header to shared memory
            # Format: timestamp (8 bytes, Q), width (4 bytes, I), height (4 bytes, I),
            #         frame_counter (8 bytes, Q), reserved (8 bytes)
            header = struct.pack('QIIQ8x', timestamp_ns, self.width, self.height, self.frame_counter)
            
            # Write to shared memory (atomic write)
            self.shm_mmap.seek(0)
            self.shm_mmap.write(header)
            self.shm_mmap.write(msg.data)
            self.shm_mmap.flush()
            
            self.frame_counter += 1
            self.frames_since_last_stats += 1
            
            # Print statistics every 2 seconds
            current_time = time.time()
            if current_time - self.last_stats_time >= 2.0:
                fps = self.frames_since_last_stats / (current_time - self.last_stats_time)
                self.get_logger().info(
                    f'Writing to SHM: {fps:.1f} fps, Frame #{self.frame_counter}'
                )
                self.last_stats_time = current_time
                self.frames_since_last_stats = 0
                
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')

    def cleanup(self):
        """Clean up shared memory resources."""
        try:
            if hasattr(self, 'shm_mmap'):
                self.shm_mmap.close()
            if hasattr(self, 'shm_fd'):
                self.shm_fd.close()
            if os.path.exists(self.shm_path):
                os.remove(self.shm_path)
            self.get_logger().info('Cleaned up shared memory')
        except Exception as e:
            self.get_logger().error(f'Error cleaning up: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RosToShmBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

