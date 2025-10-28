# 📖 使用示例

本文档提供完整的使用示例和测试场景。

---

## 🎯 场景 1：基础视频传输（摄像头 → 浏览器）

### 步骤

**1. 在 Orin 上启动所有服务：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator
./scripts/start-all.sh
```

**2. 在另一台电脑的浏览器访问：**

```
http://192.168.3.57:8080/controller.html
```

**3. 点击"连接挖掘机"按钮**

**4. 看到视频画面！** 🎉

---

## 🎮 场景 2：DataChannel 控制测试

### 步骤

**1. 启动挖掘机程序：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator
./bin/excavator \
  -signaling ws://192.168.3.57:8090/ws \
  -camera /dev/video0
```

**2. 打开浏览器控制页面并连接**

**3. 打开浏览器开发者工具（F12）**

**4. 在 Console 中测试 DataChannel：**

```javascript
// 测试方向盘控制
sendControl({
  type: "analog",
  v: { rotation: 0.5, throttle: 0.3 },
  t: Date.now()
});

// 测试档位切换
sendControl({
  type: "gear",
  gear: "D",
  t: Date.now()
});

// 测试大臂和铲斗
sendControl({
  type: "analog",
  v: { boom: 0.8, bucket: -0.5 },
  t: Date.now()
});
```

**5. 在 Orin 终端查看日志：**

```
📩 收到控制消息: {"type":"analog","v":{"rotation":0.5,"throttle":0.3},...}
```

---

## 🤖 场景 3：ROS2 集成测试

### 前提条件

确保 ROS2 已安装并配置：

```bash
source /opt/ros/humble/setup.bash  # 或其他版本
```

### 步骤

**1. 测试 ROS2 环境：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator
./scripts/test-ros2.sh
```

**2. 启动 ROS2 话题监听（新终端）：**

```bash
ros2 topic echo /controls/teleop std_msgs/msg/String
```

**3. 启动挖掘机程序（启用 ROS2）：**

```bash
./bin/excavator \
  -signaling ws://192.168.3.57:8090/ws \
  -camera /dev/video0 \
  -enable-ros2 true \
  -ros2-control-topic /controls/teleop
```

**4. 在浏览器发送控制指令**

**5. 在 ROS2 监听终端应该看到消息**（当前需要手动实现 publishControlToROS2）

---

## 🎥 场景 4：从 ROS2 获取视频（开发中）

### 概念流程

```
ROS2 摄像头节点
    │
    ├─ /camera_front_wide (sensor_msgs/Image)
    │
    ▼
挖掘机程序 (Go)
    │
    ├─ 订阅 ROS2 话题
    ├─ 图像格式转换
    ├─ H.264 硬件编码
    │
    ▼
WebRTC 视频流
    │
    ▼
浏览器显示
```

### 当前状态

⏳ **开发中** - 框架已就绪，需要实现：
1. ROS2 Image 消息解析
2. 图像格式转换（BGR/RGB → NV12/I420）
3. GStreamer 管道集成

---

## 🔧 场景 5：本机快速测试

最简单的测试方式：

```bash
# 一键启动测试
cd /home/orin64/MyCode/PionWebrtc/excavator
./scripts/test-system.sh

# 然后在浏览器打开（文件路径）
file:///home/orin64/MyCode/PionWebrtc/excavator/web/controller.html
```

---

## 📊 场景 6：性能监控

### 查看视频流统计

在浏览器 Console：

```javascript
// 获取 WebRTC 统计信息
peerConnection.getStats().then(stats => {
  stats.forEach(report => {
    if (report.type === 'inbound-rtp' && report.kind === 'video') {
      console.log('视频帧率:', report.framesPerSecond);
      console.log('码率:', report.bytesReceived * 8 / report.timestamp, 'bps');
      console.log('丢包:', report.packetsLost);
    }
  });
});
```

### 查看 GStreamer 性能

```bash
# 启动时启用调试
export GST_DEBUG=3
./bin/excavator -signaling ws://...

# 或查看系统资源
watch -n 1 'ps aux | grep excavator'
```

---

## 🛠️ 场景 7：故障排查

### 问题 1：连接成功但黑屏

**检查摄像头：**

```bash
v4l2-ctl --device=/dev/video0 --all
ls -l /dev/video*

# 测试摄像头
gst-launch-1.0 v4l2src device=/dev/video0 ! videoconvert ! autovideosink
```

**检查 GStreamer 日志：**

```bash
tail -f /tmp/excavator.log
```

### 问题 2：DataChannel 未打开

**检查浏览器 Console：**

```javascript
dataChannel.readyState  // 应该是 "open"
```

**检查 Orin 日志：**

```bash
grep "DataChannel" /tmp/excavator.log
```

应该看到：
```
✅ DataChannel 已打开
```

### 问题 3：ROS2 消息未发布

**检查 ROS2 域 ID：**

```bash
echo $ROS_DOMAIN_ID
# 如果为空，设置为 0
export ROS_DOMAIN_ID=0
```

**检查话题是否创建：**

```bash
ros2 topic list | grep teleop
```

---

## 📝 完整命令参考

### 挖掘机程序参数

```bash
./bin/excavator \
  -signaling <信令服务器地址> \      # 必填
  -camera <摄像头设备> \              # 默认 /dev/video0
  -width <视频宽度> \                 # 默认 640
  -height <视频高度> \                # 默认 480
  -fps <帧率> \                       # 默认 30
  -enable-ros2 <true|false> \         # 默认 false
  -ros2-image-topic <话题名> \        # 默认 /camera_front_wide
  -ros2-control-topic <话题名>        # 默认 /controls/teleop
```

### 完整示例

```bash
# 基础使用（仅 WebRTC）
./bin/excavator -signaling ws://192.168.3.57:8090/ws

# 高清视频
./bin/excavator \
  -signaling ws://192.168.3.57:8090/ws \
  -width 1920 -height 1080 -fps 30

# 启用 ROS2
./bin/excavator \
  -signaling ws://192.168.3.57:8090/ws \
  -enable-ros2 true \
  -ros2-control-topic /controls/teleop

# 从 ROS2 获取视频（开发中）
./bin/excavator \
  -signaling ws://192.168.3.57:8090/ws \
  -video-source ros2 \
  -ros2-image-topic /camera_front_wide \
  -enable-ros2 true
```

---

## 🚀 快速脚本

### 一键启动所有服务

```bash
./scripts/start-all.sh
```

### 停止所有服务

```bash
./scripts/kill-all.sh
```

### 查看实时日志

```bash
# 信令服务器
tail -f /tmp/signaling.log

# 挖掘机程序
tail -f /tmp/excavator.log

# Web 服务器
tail -f /tmp/webserver.log
```

---

## 💡 提示和技巧

### 1. 减少延迟

```bash
# 使用更高的码率和更低的分辨率
./bin/excavator \
  -width 640 -height 480 -fps 60
```

### 2. 网络不稳定时

- 降低分辨率：640x480 或更低
- 降低帧率：15-20fps
- 使用 STUN/TURN 服务器

### 3. CPU 占用过高

- 确保使用 Nvidia 硬件编码（`nvv4l2h264enc`）
- 降低码率或分辨率
- 检查 GStreamer 管道配置

---

## 🔗 相关文档

- [README.md](./README.md) - 项目概述和快速开始
- [ROS2_INTEGRATION.md](./ROS2_INTEGRATION.md) - ROS2 集成详细说明
- [架构文档](./README.md#架构说明) - 系统架构说明

---

## 🎓 学习资源

### WebRTC
- [MDN WebRTC API](https://developer.mozilla.org/en-US/docs/Web/API/WebRTC_API)
- [Pion WebRTC 示例](https://github.com/pion/webrtc/tree/master/examples)

### GStreamer
- [GStreamer 文档](https://gstreamer.freedesktop.org/documentation/)
- [Nvidia Jetson 多媒体 API](https://docs.nvidia.com/jetson/l4t-multimedia/)

### ROS2
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [sensor_msgs 消息类型](https://docs.ros2.org/latest/api/sensor_msgs/)

---

**有问题？欢迎提 Issue 或查看文档！** 🚀

