# Excavator WebRTC Bridge v3.0 (Shared Memory Solution)

这是一个**超低延迟**的 WebRTC 视频流和远程控制解决方案，专为 Isaac Sim / ROS2 仿真环境设计。通过**共享内存 + 进程隔离**架构，实现了零库冲突、硬件加速编码和双向实时控制。

## 🏛️ 核心架构 (v3.0 - Shared Memory)

本方案采用**共享内存**作为数据传输通道，Go 作为 WebRTC 客户端，通过进程隔离实现了极致的稳定性和性能：

- **视频流**: ROS2 → 共享内存 → GStreamer 硬编码 → Go → WebRTC → 浏览器
- **音频流 (上行)**: USB 麦克风 → GStreamer Opus 编码 → UDP → Go → WebRTC → 浏览器
- **音频流 (下行)**: 浏览器麦克风 → WebRTC → Go → UDP → GStreamer → 扬声器
- **控制流**: 浏览器 → WebRTC → Go → ROS2 发布器 → ROS2 话题

```
Isaac Sim / ROS2 仿真
         ↓
    /stitched_image (sensor_msgs/Image, rgb8)
         ↓
┌─────────────────────────────┐
│ ros_to_shm.py (ROS2 节点)   │
│ - 订阅视频话题                │
│ - 写入共享内存                │
└──────────┬──────────────────┘
           │ 共享内存 (/dev/shm)
           ↓
┌─────────────────────────────┐        云端信令服务器
│ shm_to_stdout.py (GStreamer)│             ↑ WebSocket
│ - nvh264enc 硬件编码         │             │
│ - 输出 H.264 到 stdout       │             │
└──────────┬──────────────────┘             │
           │ stdout/stdin 管道               │
           ↓                                │
┌─────────────────────────────────────┐    │
│ Go WebRTC 客户端 (main.go)           │←───┘
│ - 读取 H.264 视频流                  │
│ - 接收浏览器控制指令                 │
│ - 转发到 ros_control_stdin.py       │
└──────────┬──────────────────────────┘
           │ stdin 管道
           ↓
┌─────────────────────────────┐
│ ros_control_stdin.py        │
│ - 读取 JSON 控制指令         │
│ - 发布到 /controls/teleop   │
└──────────┬──────────────────┘
           ↓
    ROS2 控制节点 (Isaac Sim)
```

## ✨ 功能特性

- **✅ 零库冲突**: ROS2 和 GStreamer 运行在独立进程，彻底避免段错误
- **✅ 硬件加速**: NVIDIA `nvh264enc` GPU 编码，CPU 占用低
- **✅ 超低延迟**: 共享内存纳秒级数据传输，端到端延迟 ~20ms
- **✅ 双向控制**: WebRTC DataChannel 实现浏览器 ↔ ROS2 实时控制
- **✅ 双向语音**: USB 麦克风上传 + 接收控制端语音播放，Opus 编码，支持实时对讲
- **✅ 动态分辨率**: 自动适配视频分辨率变化（支持拼接图像）
- **✅ 高稳定性**: Go 主进程管理 Python 子进程，自动重启机制
- **✅ 自动重连**: WebSocket 断开后自动重连信令服务器，支持指数退避
- **✅ 易于部署**: 一键启动脚本，自动编译和环境检测

## 📦 部署与设置

### 通用依赖

- Ubuntu 22.04
- ROS2 Humble
- Go 1.18+
- Python 3.10+ (`rclpy`, `pygobject`)

---

### A) NVIDIA Jetson (Orin/Xavier) 平台指南

**这是本项目的默认和最佳实践平台。**

#### 1. 安装 GStreamer 依赖

JetPack 5.x/6.x (Orin/Xavier) 通常已包含所有必要的 GStreamer 插件。如果缺失，请安装：
```bash
sudo apt update
sudo apt install python3-gi python3-gst-1.0 gir1.2-gst-1.0 gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```
验证硬件编码器是否存在（JetPack 5/6 使用 `nvv4l2h264enc`）：
```bash
gst-inspect-1.0 nvv4l2h264enc
```

#### 2. 配置信令服务器

修改 `web/controller.html` 文件，将其中的 `SIGNALING_SERVER` 地址指向你的云服务器 IP。
```javascript
// web/controller.html
const SIGNALING_SERVER = 'ws://111.186.56.118:8090/ws'; // <-- 修改这里
```

#### 3. 构建 Go 程序

脚本会自动处理编译，你也可以手动执行一次以确保依赖正确。
```bash
# 在项目根目录
go mod tidy
```

---

### B) 标准 x86 Ubuntu 平台指南

#### 1. 安装 GStreamer 依赖

你需要安装包含 `x264enc` (软件编码器) 的插件包。
```bash
sudo apt update
sudo apt install python3-gi python3-gst-1.0 gir1.2-gst-1.0 gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly

sudo apt install python3-gi python3-gst-1.0 gir1.2-gstreamer-1.0 gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```
- **(可选) 如果你有 NVIDIA 桌面显卡**:
  - 安装最新的 NVIDIA 驱动和 CUDA Toolkit。
  - 确保 GStreamer 的 `nvh264enc` 插件可用 (通常在 `gstreamer1.0-plugins-bad` 中)。
  - 验证编码器: `gst-inspect-1.0 x264enc` 或 `gst-inspect-1.0 nvh264enc`。

#### 2. **[关键]** 修改摄像头发布节点

打开 `scripts/ros2_h264_camera_publisher.py` 文件，将其中的 GStreamer `pipeline_str` 替换为适合 x86 的版本。

**替换前的 Jetson 版本:**
```python
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
```

**替换为 x86 (软件编码 `x264enc`) 版本:**
```python
        pipeline_str = (
            f"v4l2src device={device} ! "
            f"video/x-raw,width={width},height={height},framerate={fps}/1 ! "
            "videoconvert ! "
            "video/x-raw,format=I420 ! "
            f"x264enc speed-preset=ultrafast tune=zerolatency bitrate={int(bitrate/1000)} key-int-max={fps} ! "
            "video/x-h264,stream-format=byte-stream,alignment=au ! "
            "h264parse config-interval=-1 ! "
            "appsink name=sink emit-signals=true max-buffers=4 drop=true sync=false"
        )
```
> **注意**: 如果你使用 NVIDIA 桌面显卡，可以将 `x264enc ...` 这一行替换为 `nvh264enc ...`，具体参数请查阅 GStreamer 文档。

#### 3. 配置信令服务器 & 构建 Go 程序
这部分与 Jetson 平台完全相同，请参考 **A)** 部分的步骤 2 和 3。


## 🚀 运行项目

你需要**两个**终端来分别启动摄像头节点和主程序。

### 步骤 1: 启动 Isaac Sim
确保您的 Isaac Sim 仿真环境正在运行，并且正在发布 `sensor_msgs/Image` 类型的图像话题 (默认为 `/stitched_image`)。

### 步骤 2: 启动桥接程序
在一个终端中，执行以下命令来启动所有后端服务 (ROS->SHM, 音频采集, GStreamer 编码, Go WebRTC):

```bash
# source ROS2 环境
source /opt/ros/humble/setup.bash

cd ~/code_ws/src/RemoteExcavator/webrtc_excavator/scripts/shm_solution

# 启动一体化脚本，并以参数形式传入信令服务器地址
# 默认使用 I420 格式。如果您的摄像头输出 RGB，请 export INPUT_FORMAT=RGB
# 默认启用音频采集（使用系统默认 USB 麦克风）
./start_all.sh ws://192.168.124.3:8090/ws
```

### 真实设备 (I420 摄像头)

本方案默认使用 `I420` 格式。如果您的摄像头输出 `RGB` 格式，请在启动前设置环境变量：

```bash
# 启动脚本时指定格式为 RGB
export INPUT_FORMAT=RGB
./start_all.sh ws://192.168.124.3:8090/ws
```

### 1080p 及动态分辨率支持

本项目完全支持 **1080p** 及其他分辨率，且支持**动态分辨率切换**。
- `ros_to_shm.py` 会自动检测输入话题的分辨率并调整共享内存大小。
- `shm_to_stdout.py` 会自动监测共享内存的分辨率变化并重启编码管线。
- 对于 1080p 视频，建议适当提高码率：
  ```bash
  export BITRATE_KBPS=6000  # 8Mbps
  ./start_all.sh ...
  ```

**音频配置选项**（可选）：

如果需要自定义音频设备或禁用音频，可以修改 Go 程序的启动参数：

```bash
# 禁用音频
./bin/excavator -signaling wss://... -enable-audio=false

# 指定音频设备
./bin/excavator -signaling wss://... -audio-device "alsa_input.usb-..."

# 调整音频比特率（默认 32kbps）
./bin/excavator -signaling wss://... -audio-bitrate 64000
```
> 你可以修改脚本内的 `SIGNALING_SERVER` 变量，或通过环境变量来覆盖它。

### 步骤 3: 连接控制端

打开你的 Web 浏览器，访问你部署在云服务器上的 `controller.html` 页面。页面加载后会自动连接信令服务器，并与 Excavator 建立 WebRTC 连接。连接成功后，你应该能看到来自 ROS2 的视频流，并且可以发送控制指令。

### (可选) 终端 3: 查看日志

日志现在位于项目根目录的 `logs/` 文件夹下：

```bash
# 查看 ROS2 到共享内存桥接的日志
tail -f ~/code_ws/src/RemoteExcavator/webrtc_excavator/logs/ros_to_shm.log

# 查看 Go 主程序（含编码器）的日志
tail -f ~/code_ws/src/RemoteExcavator/webrtc_excavator/logs/go_webrtc.log
```

## 🛑 停止所有进程

我们提供了一个方便的脚本来清理所有相关的后台进程。

```bash
cd /path/to/your/project/excavator/scripts
./kill-all.sh
```
该脚本会停止 Go 程序、Python 桥接和 Python 摄像头发布节点，并尝试释放摄像头设备。

