#!/bin/bash
# Kills all related processes for the excavator project.

echo "🛑 Stopping all excavator-related processes..."

# --- Shared Memory Solution (Recommended) ---
echo "  -> Stopping SHM to stdout encoder (shm_to_stdout.py)..."
pkill -f "shm_to_stdout.py" || echo "     (not running)"
if [ -f ../logs/shm_to_stdout.pid ]; then
    rm ../logs/shm_to_stdout.pid
fi

echo "  -> Stopping ROS2 control stdin bridge (ros_control_stdin.py)..."
pkill -f "ros_control_stdin.py" || echo "     (not running)"

echo "  -> Stopping Audio Streamer (audio_streamer.py)..."
pkill -f "audio_streamer.py" || echo "     (not running)"

echo "  -> Stopping Audio Player (audio_player.py)..."
pkill -f "audio_player.py" || echo "     (not running)"

echo "  -> Stopping Shared Memory to H.264 Publisher (shm_to_h264_publisher.py)..."
pkill -f "shm_to_h264_publisher.py" || echo "     (not running)"
if [ -f ../logs/shm_to_h264.pid ]; then
    rm ../logs/shm_to_h264.pid
fi

echo "  -> Stopping ROS to Shared Memory Bridge (ros_to_shm.py)..."
pkill -f "ros_to_shm.py" || echo "     (not running)"
if [ -f ../logs/ros_to_shm.pid ]; then
    rm ../logs/ros_to_shm.pid
fi

# Clean up shared memory file
echo "  -> Cleaning up shared memory..."
rm -f /dev/shm/isaac_rgb_buffer

# --- FFmpeg Solution (Backup) ---
echo "  -> Stopping FFmpeg H.264 Publisher (ros2_ffmpeg_h264.py)..."
pkill -f "ros2_ffmpeg_h264.py" || echo "     (not running)"
if [ -f ../logs/ros2_ffmpeg_h264.pid ]; then
    rm ../logs/ros2_ffmpeg_h264.pid
fi

# --- Original Processes from README ---
echo "  -> Stopping Go WebRTC bridge (excavator)..."
if [ -f ../logs/excavator.pid ]; then
    kill $(cat ../logs/excavator.pid) 2>/dev/null || true
    rm ../logs/excavator.pid
else
    pkill -f "excavator" || echo "     (not running)"
fi

echo "  -> Stopping physical camera publisher (ros2_h264_camera_publisher.py)..."
pkill -f "ros2_h264_camera_publisher.py" || echo "     (not running)"

echo "✅ Cleanup complete."

