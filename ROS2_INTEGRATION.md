# 🤖 ROS2 集成说明

本文档说明如何将挖掘机控制系统与 ROS2 集成。

## 📋 功能概述

### 已实现功能

✅ **DataChannel 控制消息接收** - 从浏览器接收控制指令  
✅ **控制消息结构定义** - 与 Python/Rust 客户端兼容的统一消息格式  
🔧 **ROS2 发布框架** - 预留了 ROS2 发布接口

### 待实现功能

⏳ **ROS2 话题发布** - 发布控制消息到 `/controls/teleop`  
⏳ **ROS2 图像订阅** - 从 `/camera_front_wide` 获取视频流  
⏳ **完整 ROS2 集成** - 使用 rclgo 原生绑定

---

## 🔧 消息格式

### WebRTC DataChannel 消息

从浏览器发送的控制消息（JSON 格式）：

```json
{
  "type": "analog",
  "v": {
    "rotation": 0.5,    // 方向盘: -1 (左) to 1 (右)
    "throttle": 0.8,    // 油门: 0 to 1
    "brake": 0.0,       // 刹车: 0 to 1
    "boom": 0.3,        // 大臂: -1 (降) to 1 (提)
    "bucket": -0.2      // 铲斗: -1 (收) to 1 (翻)
  },
  "t": 1698765432000
}
```

或档位切换消息：

```json
{
  "type": "gear",
  "gear": "D",  // P, R, N, D
  "t": 1698765432000
}
```

### ROS2 话题消息

#### 1. 控制话题 `/controls/teleop`

**消息类型**: `std_msgs/msg/String`

**消息内容**（JSON 字符串）：

```json
{
  "rotation": 0.5,
  "brake": 0.0,
  "throttle": 0.8,
  "gear": "D",
  "boom": 0.3,
  "bucket": -0.2,
  "left_track": 0.0,
  "right_track": 0.0,
  "swing": 0.0,
  "stick": 0.0,
  "device_type": "wheel_loader",
  "timestamp": 1698765432000
}
```

#### 2. 视频话题 `/camera_front_wide`

**消息类型**: `sensor_msgs/msg/Image`

**消息字段**：
- `header`: 时间戳和坐标系
- `height`: 图像高度
- `width`: 图像宽度
- `encoding`: 图像编码（如 "bgr8", "rgb8"）
- `data`: 原始图像数据

---

## 🚀 快速开始

### 方案 1：使用命令行工具（推荐用于测试）

#### 1. 启动挖掘机程序（启用 ROS2）

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator
./bin/excavator \
  -signaling ws://192.168.3.57:8090/ws \
  -camera /dev/video0 \
  -enable-ros2 true
```

#### 2. 测试 ROS2 话题

**监听控制消息：**

```bash
ros2 topic echo /controls/teleop std_msgs/msg/String
```

**手动发送测试消息：**

```bash
ros2 topic pub --once /controls/teleop std_msgs/msg/String \
  "{data: '{\"rotation\":0.5,\"throttle\":0.8,\"gear\":\"D\"}'}"
```

**查看视频话题：**

```bash
ros2 topic list | grep camera
ros2 topic info /camera_front_wide
ros2 topic hz /camera_front_wide
```

---

### 方案 2：使用 ROS2 视频源

如果你的系统中有 ROS2 摄像头节点发布视频，可以从 ROS2 获取视频：

```bash
./bin/excavator \
  -signaling ws://192.168.3.57:8090/ws \
  -video-source ros2 \
  -ros2-image-topic /camera_front_wide \
  -enable-ros2 true
```

**注意**：此功能需要进一步开发，当前版本使用摄像头作为视频源。

---

## 📦 完整 ROS2 实现（待开发）

### 当前状态

- ✅ 消息类型定义完成（`pkg/ros2/types.go`）
- ✅ 客户端框架完成（`pkg/ros2/client.go`）
- ✅ 发布/订阅接口定义（`pkg/ros2/pubsub.go`）
- ⏳ GStreamer + ROS2 图像转换（待实现）
- ⏳ 完整的控制消息路由（待实现）

### 使用 rclgo 的完整实现

**安装依赖：**

```bash
# 确保 ROS2 已安装
source /opt/ros/humble/setup.bash  # 或其他版本

# 安装 rclgo（Go 的 ROS2 客户端库）
go get github.com/tiiuae/rclgo/pkg/rclgo
```

**修改 go.mod：**

```go
require (
    github.com/tiiuae/rclgo v0.0.0-20230101000000-xxxxx
    // ... 其他依赖
)
```

### 代码示例

#### 发布控制消息到 ROS2

```go
package main

import (
    "context"
    "log"
    "excavator/pkg/ros2"
)

func publishControlToROS2(data []byte) {
    // 解析 WebRTC 消息
    var ctrlMsg ros2.ControlMessage
    if err := json.Unmarshal(data, &ctrlMsg); err != nil {
        log.Printf("⚠️ 解析控制消息失败: %v", err)
        return
    }
    
    // 转换为统一格式
    unifiedMsg := ctrlMsg.ToUnifiedControl()
    
    // 发布到 ROS2
    client, _ := ros2.NewSimpleClient()
    defer client.Close()
    
    if err := client.PublishControl("/controls/teleop", unifiedMsg); err != nil {
        log.Printf("❌ 发布到 ROS2 失败: %v", err)
    } else {
        log.Printf("✅ 已发布控制消息到 ROS2")
    }
}
```

#### 从 ROS2 订阅图像

```go
func subscribeROS2Image(videoTrack *webrtc.TrackLocalStaticSample) {
    client, _ := ros2.NewSimpleClient()
    
    // 订阅图像话题
    client.SubscribeImage("/camera_front_wide", func(img *ros2.ROS2Image) {
        // 转换 ROS2 图像为 H.264
        // 这需要额外的图像处理和编码步骤
        
        // 简化示例：直接发送（实际需要编码）
        videoTrack.WriteSample(media.Sample{
            Data:     img.Data,
            Duration: time.Millisecond * 33, // ~30fps
        })
    })
    
    log.Printf("✅ 已订阅 ROS2 图像话题")
}
```

---

## 🔄 完整工作流程

```
┌─────────────┐      WebRTC       ┌──────────────┐
│   浏览器     │ ◄──────视频───────│  挖掘机 Orin  │
│  (控制端)    │ ─────DataChannel──►│  Go 程序     │
└─────────────┘    (控制指令)      └───────┬──────┘
                                           │
                                           │ ROS2
                                           │ /controls/teleop
                                           ▼
                                  ┌────────────────┐
                                  │  ROS2 节点      │
                                  │  (Python/C++)  │
                                  └────────┬───────┘
                                           │
                                           ▼
                                  ┌────────────────┐
                                  │  实际硬件控制   │
                                  │  (CAN/Serial)  │
                                  └────────────────┘
```

---

## 🧪 测试步骤

### 1. 测试 DataChannel 控制消息接收

1. 启动挖掘机程序：
   ```bash
   ./bin/excavator -signaling ws://192.168.3.57:8090/ws -enable-ros2 true
   ```

2. 打开浏览器控制页面

3. 在浏览器控制台发送测试消息：
   ```javascript
   // 发送模拟控制指令
   dataChannel.send(JSON.stringify({
     type: "analog",
     v: { rotation: 0.5, throttle: 0.8 },
     t: Date.now()
   }));
   ```

4. 查看 Orin 终端，应该看到：
   ```
   📩 收到控制消息: {"type":"analog",...}
   🔄 [TODO] 发布控制消息到 ROS2: ...
   ```

### 2. 测试 ROS2 话题发布（手动）

在另一个终端：

```bash
# 启动 ROS2 监听
ros2 topic echo /controls/teleop std_msgs/msg/String
```

---

## 📝 下一步开发计划

### 优先级 1：控制消息发布

- [ ] 实现完整的 `publishControlToROS2` 函数
- [ ] 使用 `ros2 topic pub` 命令行工具（临时方案）
- [ ] 或使用 rclgo 实现原生发布（长期方案）

### 优先级 2：ROS2 视频源支持

- [ ] 订阅 `/camera_front_wide` 话题
- [ ] ROS2 Image → OpenCV/GStreamer 转换
- [ ] H.264 硬件编码集成

### 优先级 3：性能优化

- [ ] 减少消息转换开销
- [ ] 零拷贝图像传输
- [ ] 控制指令队列管理

---

## 🐛 常见问题

### Q: 提示 "ros2: command not found"

**A:** 确保已安装 ROS2 并 source 了环境：

```bash
source /opt/ros/humble/setup.bash
# 添加到 ~/.bashrc 永久生效
```

### Q: 话题发布失败

**A:** 检查 ROS2 域 ID 是否一致：

```bash
echo $ROS_DOMAIN_ID
# 或设置：
export ROS_DOMAIN_ID=0
```

### Q: 视频从 ROS2 获取但黑屏

**A:** 检查图像编码格式：

```bash
ros2 topic info /camera_front_wide --verbose
# 确认 encoding 字段（bgr8, rgb8, etc.）
```

---

## 📚 参考资料

- [Pion WebRTC 文档](https://github.com/pion/webrtc)
- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [rclgo GitHub](https://github.com/tiiuae/rclgo)
- [sensor_msgs/Image 消息定义](https://docs.ros2.org/latest/api/sensor_msgs/msg/Image.html)

---

## 💡 贡献指南

如果你想完善 ROS2 集成，请关注：

1. `pkg/ros2/` - ROS2 相关类型和客户端
2. `cmd/excavator/main.go` - 主程序入口
3. `publishControlToROS2` 函数 - 控制消息发布实现

欢迎提交 PR 或 Issue！🚀

