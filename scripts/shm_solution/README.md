# 共享内存 + GStreamer 方案

## 概述

这是为 Isaac Sim 视频流设计的**终极解决方案**，运行角色明确：

- **本机（挖掘机端 / Edge）职责**：订阅 Isaac Sim 摄像头话题，进行 H.264 编码，并作为 WebRTC 客户端与云端信令建立连接，将视频推送给浏览器观看者。
- **云端职责**：仅提供信令服务器（已在云端启动）。本机无需、也不应启动任何信令服务。

方案通过**进程隔离**和**共享内存**实现了：

- ✅ **零段错误**：ROS2 和 GStreamer 运行在独立进程中，彻底避免库冲突
- ✅ **硬件加速**：安全使用 NVIDIA `nvh264enc` 进行 GPU 编码
- ✅ **超低延迟**：共享内存提供纳秒级的进程间数据传输
- ✅ **高性能**：接近 Jetson 平台的原生性能

## 架构

### 视频流（下行：挖掘机 → 浏览器）

```
                                   云端（已运行信令服务）
                                 ┌───────────────────────┐
                                 │   WebSocket 信令服务器 │
                                 └───────────▲───────────┘
                                             │(信令)

挖掘机端（本机 / Edge）
Isaac Sim (1280x720 RGB @ 20fps)
         ↓
    /stitched_image (sensor_msgs/Image)
         ↓
┌─────────────────────────────────────┐
│  ros_to_shm.py (纯 ROS2 节点)        │
│  - 订阅原始图像                       │
│  - 写入共享内存 (/dev/shm)           │
└─────────────────────────────────────┘
         ↓ (共享内存 - 纳秒级延迟)
┌─────────────────────────────────────┐
│  shm_to_stdout.py (纯 GStreamer)     │
│  - 从共享内存读取                     │
│  - nvh264enc 硬件编码                │
│  - 通过 stdout 输出 H.264 码流        │
└─────────────────────────────────────┘
         ↓ (子进程 stdout)
┌─────────────────────────────────────┐
│  Go WebRTC Bridge (本机客户端)       │
│  - 读取子进程 stdout 的 H.264         │
│  - 通过云端信令建立 P2P/WebRTC       │
└───────────┬───────────────┘
            │(WebRTC)
            ↓
        浏览器 (远端观看者)
```

### 控制指令（上行：浏览器 → 挖掘机）

```
浏览器 (远端控制端)
  ↓ WebRTC DataChannel (JSON 字符串)
┌─────────────────────────────────────┐
│  Go WebRTC Bridge (本机客户端)       │
│  - 接收 DataChannel 消息             │
│  - 写入 Python 脚本的 stdin         │
└───────────┬───────────────┘
            │(stdin 管道)
            ↓
┌─────────────────────────────────────┐
│  ros_control_stdin.py (纯 ROS2 节点)│
│  - 从 stdin 读取 JSON 字符串         │
│  - 验证 JSON 格式                    │
│  - 发布到 ROS2 话题                  │
└───────────┬───────────────┘
            │(ROS2 发布)
            ↓
    /controls/teleop (std_msgs/String)
            ↓
    ROS2 控制节点（执行控制指令）
```

## 文件说明

### 核心脚本（4 个）

1. **`ros_to_shm.py`**（视频生产者）
   - 纯 ROS2 节点（仅 `rclpy`）
   - 订阅 `/stitched_image`（`sensor_msgs/Image`，rgb8）
   - 将原始 RGB 写入共享内存 `/dev/shm/isaac_rgb_buffer`（带 32B 帧头）

2. **`shm_to_stdout.py`**（视频编码器，推荐）
   - 纯 GStreamer 程序（仅 `gi`）
   - 从共享内存读帧，经 `nvh264enc` 硬编
   - 通过 `fdsink fd=1` 将 H.264 码流输出到 stdout，供 Go 子进程直接读取
   - 优点：完全进程隔离，最低延迟，Go 端无需改动读取逻辑

3. **`ros_control_stdin.py`**（控制桥接）
   - 纯 ROS2 节点（仅 `rclpy`）
   - 从 stdin 读取 JSON 控制指令
   - 验证 JSON 格式并发布到 `/controls/teleop`（`std_msgs/String`）
   - **自动启动**：Go 客户端会自动启动此脚本

4. **`shm_to_h264_publisher.py`**（可选/调试）
   - GStreamer + ROS2 混合节点：从共享内存读取、硬编、并发布到 `/camera/image_h264`
   - 适用于需要在 ROS 可视化工具中直接消费 H.264 的场景
   - 注意：此脚本与 `rclpy` 同进程使用 GStreamer，可能在某些环境下引发库冲突；推荐生产环境使用 `shm_to_stdout.py`

### 启动脚本

1. **`start_ros_to_shm.sh`** - 启动 ROS2→共享内存桥接
2. **`start_shm_to_h264.sh`** - 启动共享内存→H.264 stdout 编码器（内部已改为启动 `shm_to_stdout.py`）
3. **`start_all.sh`** - 一键启动（先桥接，再编码器）

## 使用方法

### 快速启动（挖掘机端/本机）

```bash
# 进入脚本目录
cd ~/code_ws/src/RemoteExcavator/webrtc_excavator/scripts/shm_solution

# 启动 ROS→SHM 桥接（仅一次）
# 默认使用 I420 格式。如果您的摄像头输出 RGB，请 export INPUT_FORMAT=RGB
./start_ros_to_shm.sh
```

```bash
# 在项目根目录编译并启动 WebRTC 客户端（指向云端信令）
cd ~/code_ws/src/RemoteExcavator/webrtc_excavator
go build -o bin/excavator ./cmd/excavator
./bin/excavator -signaling ws://111.186.56.118:8090/ws
```

说明：

- Go 客户端会自动拉起 `shm_to_stdout.py`（视频编码器）和 `ros_control_stdin.py`（控制桥接）。
- **不要**在本机启动信令服务器；信令服务已在云端运行，上面的 `-signaling` 需指向云端地址。
- 若你更偏好一键调试，也可使用 `./start_all.sh` 本地先起两个生产者进程，再启动 Go 客户端。

### 分步启动（用于调试）

```bash
# 步骤 1: 启动 ROS2 到共享内存桥接 (默认 I420)
./start_ros_to_shm.sh

# 等待 2 秒，让共享内存文件创建完成
sleep 2

# 步骤 2: 启动共享内存到 H.264 发布器
# 注意：这通常由 Go 客户端自动启动，仅调试时手动运行
./start_shm_to_h264.sh
```

### 监控日志

日志现在位于项目根目录的 `logs/` 文件夹下：

```bash
# 监控 ROS2 到共享内存桥接的日志
tail -f ~/code_ws/src/RemoteExcavator/webrtc_excavator/logs/ros_to_shm.log

# 监控 Go WebRTC 客户端（含编码器输出）的日志
tail -f ~/code_ws/src/RemoteExcavator/webrtc_excavator/logs/go_webrtc.log
```

### 检查状态

```bash
# 列出所有话题
ros2 topic list

# 检查共享内存文件是否存在
ls -lh /dev/shm/isaac_rgb_buffer

# 检查视频话题是否有数据
ros2 topic hz /stitched_image

# 检查控制话题是否有数据（需要浏览器发送控制指令）
ros2 topic hz /controls/teleop

# 监听控制话题内容
ros2 topic echo /controls/teleop std_msgs/msg/String
```

### 停止所有进程

```bash
cd ~/code_ws/excavator/scripts
./kill-all.sh
```

## 参数配置

### `ros_to_shm.py` 参数

可以通过 ROS2 参数覆盖默认值：

```bash
python3 ros_to_shm.py \
    --ros-args \
    -p input_topic:=/stitched_image \
    -p shm_path:=/dev/shm/isaac_rgb_buffer \
    -p width:=1280 \
    -p height:=720
```

### `ros_control_stdin.py` 参数

Go 客户端会自动启动此脚本，但也可以手动测试：

```bash
# 手动测试控制桥接（从 stdin 读取并发布到 ROS2）
echo '{"left_track":0.5,"right_track":0.5,"boom":0.3,"bucket":-0.2,"device_type":"excavator","timestamp":1234567890}' | \
  python3 ros_control_stdin.py --ros-args -p control_topic:=/controls/teleop
```

### `shm_to_h264_publisher.py` 参数

```bash
python3 shm_to_h264_publisher.py \
    --ros-args \
    -p output_topic:=/camera/image_h264 \
    -p shm_path:=/dev/shm/isaac_rgb_buffer \
    -p width:=1280 \
    -p height:=720 \
    -p fps:=20 \
    -p bitrate_kbps:=4000
```

## 控制指令消息格式

前端通过 WebRTC DataChannel 发送的 JSON 控制消息格式：

```json
{
  "rotation": 0.0,
  "brake": 0.0,
  "throttle": 0.0,
  "gear": "N",
  "boom": 0.0,
  "bucket": 0.0,
  "left_track": 0.0,
  "right_track": 0.0,
  "swing": 0.0,
  "stick": 0.0,
  "device_type": "excavator",
  "timestamp": 1698765432000
}
```

**字段说明**：
- `rotation`: 旋转控制 (-1.0 到 1.0)
- `brake`: 刹车 (0.0 到 1.0)
- `throttle`: 油门 (0.0 到 1.0)
- `gear`: 档位 ("N", "D", "R")
- `boom`: 动臂控制 (-1.0 到 1.0)
- `bucket`: 铲斗控制 (-1.0 到 1.0)
- `left_track`: 左履带 (-1.0 到 1.0)
- `right_track`: 右履带 (-1.0 到 1.0)
- `swing`: 回转控制 (-1.0 到 1.0)
- `stick`: 斗杆控制 (-1.0 到 1.0)
- `device_type`: 设备类型 ("excavator", "wheel_loader" 等)
- `timestamp`: 时间戳（毫秒）

**ROS2 话题格式**：
- **话题名称**: `/controls/teleop`
- **消息类型**: `std_msgs/String`
- **消息内容**: JSON 字符串（与前端发送的格式相同）
- **QoS**: `depth=1, BEST_EFFORT, KEEP_LAST`

## 技术细节

### 共享内存格式

共享内存文件结构（32 字节头 + 图像数据）：

```
Offset | Size | Field
-------|------|------------------
0      | 8    | timestamp_ns (uint64)
8      | 4    | width (uint32)
12     | 4    | height (uint32)
16     | 8    | frame_counter (uint64)
24     | 8    | reserved (padding)
32     | N    | RGB image data (width * height * 3 bytes)
```

### GStreamer 管线（shm_to_stdout.py）

```
appsrc 
  → video/x-raw,format=RGB 
  → videoconvert 
  → video/x-raw,format=I420 
  → nvh264enc (硬件编码)
  → h264parse 
  → fdsink fd=1 (输出到 stdout)
```

### QoS 配置

仅 `ros_to_shm.py` 是 ROS2 节点，推荐 QoS：

- `depth=1`：只保留最新的一帧
- `reliability=BEST_EFFORT`：允许丢帧，优先低延迟
- `history=KEEP_LAST`：不保留历史数据

## 故障排查

### 问题：共享内存文件未创建

**症状**：`shm_to_h264_publisher.py` 报错 "Shared memory file not found"

**解决**：
1. 确保 `ros_to_shm.py` 已启动且没有错误
2. 检查 `/stitched_image` 话题是否有数据：`ros2 topic hz /stitched_image`
3. 检查日志：`tail -f ~/code_ws/excavator/logs/ros_to_shm.log`

### 问题：段错误 (Segmentation Fault)

**症状**：脚本崩溃并显示 "段错误 (核心已转储)"

**解决**：
1. 使用推荐组合：`ros_to_shm.py`（仅 rclpy）+ `shm_to_stdout.py`（仅 gi）
2. 避免在同一进程内同时加载 rclpy 与 GStreamer
3. 重新启动所有进程：`./kill-all.sh && ./start_all.sh`

### 问题：GStreamer 找不到 nvh264enc

**症状**：日志显示 "no element 'nvh264enc'"

**解决**：
```bash
# 安装 GStreamer bad 插件
sudo apt-get install gstreamer1.0-plugins-bad

# 验证插件是否可用
gst-inspect-1.0 nvh264enc
```

### 问题：帧率低或延迟高

**可能原因**：
1. Isaac Sim 发布频率低 - 检查 `ros2 topic hz /stitched_image`
2. GPU 负载过高 - 运行 `nvidia-smi` 检查
3. QoS 不匹配 - 确保两个节点使用相同的 QoS 配置

### 问题：无法建立 WebRTC 连接（信令相关）

**症状**：浏览器看不到视频，或日志长时间停留在"等待候选/等待应答"。

**解决**：
1. 确认 `-signaling` 指向的是云端信令服务器，例如 `ws://your.cloud.ip:8090/ws`。
2. 确认本机到云端的网络可达：`curl -v ws://your.cloud.ip:8090/ws`（或使用 `wscat`）。
3. 防火墙/NAT：允许本机对云端信令的出站访问；必要时配置 TURN 服务器以保证弱网/NAT 情况下连通性。
4. 查看本机 Go 客户端与 `shm_to_stdout.py` 日志，确认编码器已启动且在产出 H.264。

### 问题：控制指令无法发布到 ROS2

**症状**：浏览器发送控制指令，但 ROS2 话题 `/controls/teleop` 没有消息。

**解决**：
1. 检查 Go 客户端日志，确认 `ros_control_stdin.py` 已启动：
   ```bash
   tail -f ~/code_ws/excavator/logs/go_webrtc.log | grep "ROS2 控制器"
   ```
2. 检查 DataChannel 是否已打开：
   ```bash
   tail -f ~/code_ws/excavator/logs/go_webrtc.log | grep "DataChannel"
   ```
3. 手动测试控制桥接脚本：
   ```bash
   echo '{"left_track":0.5,"device_type":"excavator","timestamp":1234567890}' | \
     python3 scripts/shm_solution/ros_control_stdin.py --ros-args -p control_topic:=/controls/teleop
   ```
4. 检查 ROS2 话题是否存在：
   ```bash
   ros2 topic list | grep controls
   ros2 topic echo /controls/teleop std_msgs/msg/String
   ```
5. 确认前端发送的 JSON 格式正确（参考下面的消息格式）。

## 性能对比

| 方案 | 延迟 | CPU 使用 | GPU 使用 | 稳定性 |
|------|------|----------|----------|--------|
| FFmpeg (backup) | ~100ms | 高 | 无 | 极高 |
| **共享内存 + GStreamer** | **~20ms** | **低** | **中** | **高** |

## 与 FFmpeg 方案的对比

| 特性 | FFmpeg 方案 | 共享内存方案 |
|------|-------------|--------------|
| 数据传输 | stdin/stdout 管道 | 共享内存 |
| 编码器 | libx264 (软件) | nvh264enc (硬件) |
| 延迟 | ~100ms | ~20ms |
| CPU 占用 | 高 | 低 |
| 稳定性 | 极高 | 高 |
| 部署复杂度 | 简单 | 中等 |

## 许可证

与主项目相同。

