# Excavator WebRTC Bridge v2.0

这是一个低延迟的 WebRTC 视频流解决方案，旨在将来自 ROS2 的视频（物理摄像头或 Isaac Sim）高效地传输到远程 Web 浏览器，并实现双向控制。

## 🏛️ 核心架构 (v2.0)

本方案采用 Go 作为 WebRTC 客户端，并通过标准的 `stdin/stdout` 管道与一个 Python 脚本进行双向通信，完全移除了对 `rclgo` 的依赖，实现了极致的解耦和稳定性。

```
+--------------------------+      +------------------+      +---------------------------+
| [ROS2 H.264 Publisher]   |----->| [ROS2 Topic]     |<-----| [Python Bridge] (Subscribe)|
| (Python, GStreamer HW/SW)|      | CompressedImage  |      +---------------------------+
+--------------------------+      +------------------+                    |  (stdout)
                                                                         | [H.264 + Timestamp]
                                                                         v
+--------------------------+      +------------------+      +---------------------------+
| [Control Application]    |----->| [ROS2 Topic]     |----->| [Python Bridge] (Publish)   |
| (e.g., teleop_twist_joy) |      | std_msgs/String  |      +---------------------------+
+--------------------------+      +------------------+                    ^  (stdin)
                                                                         | [Control JSON]
                                                                         |
       +-----------------------------------------------------------------+
       |
       v
+-------------------------------------------------------------+      +-----------------+
| [Excavator - Go WebRTC Client]                              |<---->| [Signaling    ] |
| - Manages Python Bridge subprocess                          |      | [Server       ] |
| - Reads H.264 from stdout, writes Control JSON to stdin     |      +-----------------+
| - Handles all WebRTC logic (PeerConnection, ICE, DataChannel)|
+-------------------------------------------------------------+
       ^
       | (WebRTC)
       v
+--------------------------+
| [Browser]                |
| (Next.js Controller)     |
+--------------------------+
```

## ✨ 功能特性

- **ROS2 Humble 集成**: 无缝接入现有 ROS2 系统。
- **低延迟视频流**: 利用 H.264 传递压缩视频流，避免重复编解码。
- **硬件加速优化**:
    - 在 **NVIDIA Jetson (Orin, Xavier)** 平台，使用 `nvv4l2h264enc` 进行硬件编码。
    - 在 **x86 PC** 平台，支持使用 `x264enc` (软件) 或 `nvh264enc` (NVIDIA 显卡) 进行编码。
- **双向控制**: 通过 WebRTC DataChannel 从浏览器发送 JSON 指令，实时控制 ROS2 节点。
- **高稳定性**: Go 主进程管理 Python 子进程，通过 `stdin/stdout` 管道通信，稳定可靠。
- **精确帧率控制**: 从 ROS2 消息头中提取精确的时间戳，用于计算 `media.Sample` 的 `Duration`，保证视频播放平滑。

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

JetPack 通常已包含所有必要的 GStreamer 插件 (`nvidia-l4t-gstreamer`)。如果缺失，请安装：
```bash
sudo apt update
sudo apt install python3-gi python3-gst-1.0 gir1.2-gst-1.0 gstreamer1.0-tools gstreamer1.0-plugins-good gstreamer1.0-plugins-bad
```
验证硬件编码器是否存在：
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

### 终端 1: 启动 ROS2 摄像头发布节点

```bash
# source ROS2 环境
source /opt/ros/humble/setup.bash

cd /path/to/your/project/excavator/scripts

# 启动摄像头发布脚本
# 它会使用 v4l2 捕获摄像头，(硬/软)编码为 H.264，然后发布到 ROS2 话题
./start-ros2-h264-camera.sh
```
> 脚本接受参数，例如: `./start-ros2-h264-camera.sh --device /dev/video1 --width 1280 --height 720`

### 终端 2: 启动 Excavator 桥接程序

```bash
# source ROS2 环境
source /opt/ros/humble/setup.bash

cd /path/to/your/project/excavator/scripts

# 启动主程序脚本
# 它会编译并运行 Go 程序，Go 程序会自动启动 Python 桥接子进程
./start-excavator-bridge.sh
```
> 你可以修改脚本内的 `SIGNALING_SERVER` 变量，或通过环境变量来覆盖它。

### 步骤 3: 连接控制端

打开你的 Web 浏览器，访问你部署在云服务器上的 `controller.html` 页面。页面加载后会自动连接信令服务器，并与 Excavator 建立 WebRTC 连接。连接成功后，你应该能看到来自 ROS2 的视频流，并且可以发送控制指令。

### (可选) 终端 3: 查看日志

```bash
# 查看 Go 主程序和 Python 桥接的日志
tail -f /path/to/your/project/excavator/logs/excavator-bridge.log

# 查看摄像头发布节点的日志
tail -f /path/to/your/project/excavator/logs/ros2-h264-camera.log
```

## 🛑 停止所有进程

我们提供了一个方便的脚本来清理所有相关的后台进程。

```bash
cd /path/to/your/project/excavator/scripts
./kill-all.sh
```
该脚本会停止 Go 程序、Python 桥接和 Python 摄像头发布节点，并尝试释放摄像头设备。

