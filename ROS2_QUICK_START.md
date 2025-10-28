# 🚀 ROS2 快速开始指南

本文档提供 ROS2 功能的快速测试和使用方法。

---

## ✅ 前提条件

1. **ROS2 已安装并配置**
   ```bash
   source /opt/ros/humble/setup.bash  # 或其他版本
   ```

2. **cv_bridge 已安装**（用于图像桥接）
   ```bash
   sudo apt install ros-humble-cv-bridge python3-opencv
   ```

3. **有视频话题在发布**
   ```bash
   ros2 topic list | grep camera
   # 应该看到 /camera_front_wide 或类似话题
   ```

---

## 🎯 功能 1：发布控制消息到 ROS2

### 测试步骤

**终端 1 - 监听 ROS2 控制话题：**

```bash
ros2 topic echo /controls/teleop std_msgs/msg/String
```

**终端 2 - 启动信令服务器：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator
./bin/signaling -addr :8090
```

**终端 3 - 启动挖掘机程序（启用 ROS2）：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator
./bin/excavator \
  -signaling ws://127.0.0.1:8090/ws \
  -camera /dev/video0 \
  -enable-ros2 true \
  -ros2-control-topic /controls/teleop
```

**终端 4 - 启动 Web 服务器：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator/web
python3 -m http.server 8080
```

**浏览器 - 发送控制指令：**

1. 打开 `http://localhost:8080/controller.html`
2. 点击 "连接挖掘机"
3. 打开浏览器 Console（F12）
4. 发送测试指令：

```javascript
// 测试方向盘和油门
dataChannel.send(JSON.stringify({
  type: "analog",
  v: { rotation: 0.5, throttle: 0.8 },
  t: Date.now()
}));

// 测试档位切换
dataChannel.send(JSON.stringify({
  type: "gear",
  gear: "D",
  t: Date.now()
}));

// 测试大臂和铲斗
dataChannel.send(JSON.stringify({
  type: "analog",
  v: { boom: 0.8, bucket: -0.5 },
  t: Date.now()
}));
```

**预期结果：**

在终端 1 中应该看到：

```yaml
data: '{"rotation":0.5,"brake":0.0,"throttle":0.8,"gear":"N",...}'
---
data: '{"rotation":0.0,"brake":0.0,"throttle":0.0,"gear":"D",...}'
---
data: '{"rotation":0.0,"brake":0.0,"throttle":0.0,"gear":"N","boom":0.8,"bucket":-0.5,...}'
---
```

在终端 3（excavator）中应该看到：

```
📩 收到控制消息: {"type":"analog","v":{"rotation":0.5,"throttle":0.8},...}
✅ 已发布控制消息到 ROS2: /controls/teleop
```

---

## 🎥 功能 2：从 ROS2 订阅视频

### 架构说明

```
ROS2 摄像头节点
    │
    ├─ /camera_front_wide (sensor_msgs/Image)
    │
    ▼
Python 图像桥接脚本
    │
    ├─ 转换为 JPEG
    ├─ UDP 发送到端口 5000
    │
    ▼
GStreamer (excavator)
    │
    ├─ udpsrc 接收
    ├─ H.264 编码
    │
    ▼
WebRTC → 浏览器
```

### 测试步骤

**终端 1 - 启动 ROS2 图像桥接：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator

# 检查 ROS2 话题
ros2 topic list | grep camera

# 启动桥接（确保有 cv_bridge）
python3 scripts/ros2_image_bridge.py /camera_front_wide 5000 80
```

输出示例：
```
🌉 ROS2 图像桥接
📡 话题: /camera_front_wide
📤 UDP: 127.0.0.1:5000
🎨 质量: 80

✅ 图像桥接已启动
📡 订阅话题: /camera_front_wide
📤 UDP 端口: 5000
🎨 JPEG 质量: 80
📹 已发送 30 帧 (45678 字节)
📹 已发送 60 帧 (45123 字节)
...
```

**终端 2 - 启动信令服务器：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator
./bin/signaling -addr :8090
```

**终端 3 - 启动挖掘机程序（使用 ROS2 视频源）：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator
./bin/excavator \
  -signaling ws://127.0.0.1:8090/ws \
  -video-source ros2 \
  -ros2-image-topic /camera_front_wide \
  -enable-ros2 true \
  -ros2-control-topic /controls/teleop
```

输出示例：
```
🚀 挖掘机端启动...
📡 连接信令服务器: ws://127.0.0.1:8090/ws
✅ 已注册为 excavator
✅ DataChannel 已打开
🔄 配置 ROS2 视频源: /camera_front_wide
⚠️  使用 ROS2 视频源需要先启动图像桥接:
   在另一个终端运行:
   cd /home/orin64/MyCode/PionWebrtc/excavator
   python3 scripts/ros2_image_bridge.py /camera_front_wide 5000

   等待桥接启动后，视频将自动接入...
📹 视频源: ROS2 话题 /camera_front_wide
⏳ 等待控制端连接...
🎬 启动 GStreamer 管道: h264
✅ h264 管道运行中
```

**终端 4 - 启动 Web 服务器：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator/web
python3 -m http.server 8080
```

**浏览器 - 查看视频：**

1. 打开 `http://localhost:8080/controller.html`
2. 点击 "连接挖掘机"
3. 应该能看到从 ROS2 话题来的视频！🎉

---

## 🔧 故障排查

### 问题 1：Python 桥接报错 "No module named 'cv_bridge'"

**解决方案：**

```bash
# 安装 cv_bridge
sudo apt install ros-humble-cv-bridge

# 或从源码编译
cd ~/ros2_ws/src
git clone https://github.com/ros-perception/vision_opencv.git
cd ~/ros2_ws
colcon build --packages-select cv_bridge
source install/setup.bash
```

### 问题 2：excavator 显示 "udpsrc: No data received"

**检查项：**

1. **Python 桥接是否运行？**
   ```bash
   ps aux | grep ros2_image_bridge
   ```

2. **UDP 端口是否正确？**
   ```bash
   # 查看 UDP 5000 端口
   sudo netstat -ulnp | grep 5000
   ```

3. **ROS2 话题是否有数据？**
   ```bash
   ros2 topic hz /camera_front_wide
   ```

### 问题 3：视频卡顿或延迟高

**优化方案：**

1. **降低 JPEG 质量**（减少带宽）
   ```bash
   python3 scripts/ros2_image_bridge.py /camera_front_wide 5000 60
   # 最后的参数 60 是 JPEG 质量（1-100）
   ```

2. **使用原始 UDP（不使用 RTP）**
   
   修改 `createROS2VideoSource` 函数：
   ```go
   // 注释掉 RTP 管道
   // pipeline := fmt.Sprintf(
   //     "udpsrc port=%d ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! videoconvert ! video/x-raw,format=I420 ! queue",
   //     udpPort,
   // )
   
   // 使用直接 JPEG 解码
   pipeline := fmt.Sprintf(
       "udpsrc port=%d ! jpegdec ! videoconvert ! video/x-raw,format=I420 ! queue",
       udpPort,
   )
   ```

3. **增加 UDP 缓冲区**
   ```go
   pipeline := fmt.Sprintf(
       "udpsrc port=%d buffer-size=65536 ! jpegdec ! videoconvert ! video/x-raw,format=I420 ! queue",
       udpPort,
   )
   ```

### 问题 4：控制消息未发布到 ROS2

**检查项：**

1. **是否启用了 ROS2？**
   ```bash
   # 确保使用 -enable-ros2 true
   ./bin/excavator ... -enable-ros2 true
   ```

2. **话题名称是否正确？**
   ```bash
   # 查看所有话题
   ros2 topic list
   
   # 应该看到 /controls/teleop（如果有消息发布）
   ```

3. **DataChannel 是否打开？**
   
   在浏览器 Console 查看：
   ```javascript
   console.log(dataChannel.readyState); // 应该是 "open"
   ```

---

## 📊 性能监控

### ROS2 话题频率

```bash
# 查看视频话题频率
ros2 topic hz /camera_front_wide

# 查看控制话题频率
ros2 topic hz /controls/teleop

# 查看话题带宽
ros2 topic bw /camera_front_wide
```

### GStreamer 调试

```bash
# 启用 GStreamer 调试
export GST_DEBUG=3
./bin/excavator ...

# 查看特定元素的调试信息
export GST_DEBUG=udpsrc:5,jpegdec:5
./bin/excavator ...
```

### 系统资源

```bash
# 查看进程资源占用
watch -n 1 'ps aux | grep -E "excavator|ros2_image_bridge"'

# 查看网络流量
sudo iftop -i lo  # 本地环回接口
```

---

## 🎉 完整测试命令

### 一键测试脚本

创建 `test-ros2-full.sh`：

```bash
#!/bin/bash

# 一键测试 ROS2 完整功能

echo "🧪 ROS2 完整功能测试"
echo ""

# 检查 ROS2
if ! command -v ros2 >/dev/null; then
    echo "❌ ROS2 未安装"
    exit 1
fi

echo "✅ ROS2 可用"

# 检查话题
if ! ros2 topic list | grep -q "/camera_front_wide"; then
    echo "⚠️  未找到 /camera_front_wide 话题"
    echo "请先启动 ROS2 摄像头节点"
    exit 1
fi

echo "✅ 视频话题存在"
echo ""

# 启动服务（需要多个终端）
echo "请按以下顺序启动服务："
echo ""
echo "1️⃣ 终端 1 - 图像桥接:"
echo "   python3 scripts/ros2_image_bridge.py"
echo ""
echo "2️⃣ 终端 2 - 信令服务器:"
echo "   ./bin/signaling -addr :8090"
echo ""
echo "3️⃣ 终端 3 - 挖掘机程序:"
echo "   ./bin/excavator -signaling ws://127.0.0.1:8090/ws -video-source ros2 -enable-ros2 true"
echo ""
echo "4️⃣ 终端 4 - Web 服务器:"
echo "   cd web && python3 -m http.server 8080"
echo ""
echo "5️⃣ 浏览器:"
echo "   http://localhost:8080/controller.html"
echo ""
```

---

## 📚 参考资料

- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [cv_bridge 教程](http://wiki.ros.org/cv_bridge/Tutorials)
- [sensor_msgs/Image](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)
- [GStreamer UDP 插件](https://gstreamer.freedesktop.org/documentation/udp/)

---

## 💡 下一步

- [ ] 优化 Python 桥接性能（使用 C++ 或 Rust 重写）
- [ ] 支持更多图像编码格式（H.264, VP8 等）
- [ ] 添加视频质量自适应调整
- [ ] 实现双向控制（ROS2 → 浏览器）

**有问题？查看 [ROS2_INTEGRATION.md](./ROS2_INTEGRATION.md) 获取更多详情！** 🚀

