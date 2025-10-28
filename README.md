# 🚜 挖掘机远程控制系统

基于 WebRTC + WebSocket 的挖掘机远程视频监控系统

## 📁 项目结构

```
excavator/
├── cmd/
│   ├── signaling/         # 信令服务器（WebSocket）
│   └── excavator/         # 挖掘机端程序（Orin）
├── web/
│   └── controller.html    # 远程控制网页
├── bin/
│   ├── signaling          # 信令服务器可执行文件
│   └── excavator          # 挖掘机程序可执行文件
└── scripts/               # 启动脚本
```

## 🚀 快速开始

### 准备工作

**需要 3 个设备：**
1. **云服务器/VPC** - 运行信令服务器
2. **装载机 Orin** - 运行挖掘机程序
3. **控制端电脑** - 打开浏览器控制

### 第一步：启动信令服务器（云服务器）

```bash
# 在云服务器上运行
cd excavator/bin
./signaling -addr :8090
```

**输出示例：**
```
🚀 信令服务器启动: :8090
📡 WebSocket 端点: ws://<服务器IP>:8090/ws
📊 状态查询: http://<服务器IP>:8090/status
```

### 第二步：启动挖掘机程序（Orin）

```bash
# 在 Orin 上运行
cd excavator/bin

# 替换 <服务器IP> 为你的信令服务器 IP
./excavator \
  -signaling ws://<服务器IP>:8090/ws \
  -camera /dev/video0 \
  -width 640 \
  -height 480 \
  -fps 30
```

**输出示例：**
```
🚀 挖掘机端启动...
📡 连接信令服务器: ws://x.x.x.x:8090/ws
✅ 已注册为 excavator
📹 摄像头: /dev/video0 (640x480@30fps)
⏳ 等待控制端连接...
```

### 第三步：打开控制页面（浏览器）

1. **编辑控制页面配置：**
   打开 `web/controller.html`，修改第 122 行：
   ```javascript
   const SIGNALING_SERVER = 'ws://<服务器IP>:8090/ws';
   ```

2. **在浏览器打开：**
   ```bash
   # 方式 1：直接双击打开 controller.html
   
   # 方式 2：通过 HTTP 服务器
   cd excavator/web
   python3 -m http.server 8080
   # 然后浏览器访问 http://localhost:8080/controller.html
   ```

3. **点击 "连接挖掘机" 按钮**

4. **看到视频画面！** 🎉

## ⚙️ 参数说明

### 信令服务器参数

```bash
./signaling -addr <地址>
```

- `-addr`: 监听地址（默认 `:8090`）

### 挖掘机程序参数

```bash
./excavator [选项]
```

- `-signaling`: 信令服务器地址（**必填**）
- `-camera`: 摄像头设备（默认 `/dev/video0`）
- `-width`: 视频宽度（默认 `640`）
- `-height`: 视频高度（默认 `480`）
- `-fps`: 视频帧率（默认 `30`）

**示例：**

```bash
# 使用 1080p @ 30fps
./excavator -signaling ws://10.0.1.100:8090/ws -width 1920 -height 1080 -fps 30

# 使用不同的摄像头
./excavator -signaling ws://10.0.1.100:8090/ws -camera /dev/video1
```

## 📊 架构说明

```
┌─────────────┐      WebSocket      ┌─────────────┐
│   控制端     │◄──────信令───────────│  云服务器    │
│  (浏览器)    │                      │ (信令服务器) │
└──────┬──────┘                      └─────┬───────┘
       │                                   │
       │         WebRTC P2P连接           │ WebSocket
       │         (音视频数据)              │   信令
       │                                   │
       └────────────────┬──────────────────┘
                       │
                ┌──────▼──────┐
                │  装载机      │
                │  (Orin)      │
                │  摄像头+编码  │
                └─────────────┘
```

### 信令流程

1. **注册阶段**
   - 控制端连接信令服务器 → 注册为 `controller`
   - 挖掘机连接信令服务器 → 注册为 `excavator`

2. **建立连接**
   - 控制端创建 Offer → 通过信令服务器发送给挖掘机
   - 挖掘机创建 Answer → 通过信令服务器发送回控制端
   - 交换 ICE 候选

3. **数据传输**
   - WebRTC P2P 连接建立
   - 视频/音频流直接传输（不经过信令服务器）

## 🔥 核心特性

✅ **自动 SDP 交换** - 无需手动复制粘贴  
✅ **Nvidia 硬件编码** - 使用 Orin 的 H.264 硬件编码器  
✅ **低延迟传输** - WebRTC P2P 直连  
✅ **自动重连** - 网络断开自动恢复  
✅ **实时日志** - 连接状态实时显示  
✅ **DataChannel 支持** - 接收浏览器控制指令  
🔧 **ROS2 集成** - 控制消息发布到 ROS2（开发中）  

## 🤖 ROS2 功能

### 已实现

✅ **DataChannel 控制接收** - 从浏览器接收控制指令  
✅ **ROS2 消息类型定义** - 兼容 Python/Rust 客户端  
✅ **ROS2 客户端框架** - 发布/订阅接口已就绪  

### 开发中

🔧 **控制消息发布** - 发布到 `/controls/teleop` 话题  
🔧 **ROS2 视频源** - 从 `/camera_front_wide` 获取视频  

### 使用示例

```bash
# 启用 ROS2 功能
./bin/excavator \
  -signaling ws://192.168.3.57:8090/ws \
  -enable-ros2 true \
  -ros2-control-topic /controls/teleop

# 监听 ROS2 控制消息（新终端）
ros2 topic echo /controls/teleop std_msgs/msg/String
```

**详细文档**: [ROS2_INTEGRATION.md](./ROS2_INTEGRATION.md) | [EXAMPLES.md](./EXAMPLES.md)

## 🛠️ 故障排查

### 控制端无法连接

1. **检查信令服务器**
   ```bash
   curl http://<服务器IP>:8090/status
   ```
   应该返回在线客户端列表

2. **检查防火墙**
   ```bash
   # 云服务器需要开放 8090 端口
   sudo ufw allow 8090/tcp
   ```

### 连接成功但黑屏

1. **检查摄像头**
   ```bash
   v4l2-ctl --device=/dev/video0 --all
   ls -l /dev/video*
   ```

2. **查看挖掘机程序日志**
   看是否有 GStreamer 错误

3. **尝试软件编码**
   修改视频源（去掉 nvvidconv）：
   ```bash
   ./excavator -signaling ws://... -video-src "v4l2src device=/dev/video0 ! videoconvert ! video/x-raw,format=I420"
   ```

### ICE 连接失败

1. **本地网络测试**（同一局域网）
   - 信令服务器和两端在同一网络
   - 不需要 STUN/TURN

2. **跨网络测试**（需要公网）
   - 需要配置 STUN 服务器
   - 或使用 TURN 中继服务器

## 📝 开发日志

- [x] 基础视频传输功能
- [x] WebSocket 信令服务器
- [x] 自动 SDP 交换
- [x] Nvidia 硬件编码支持
- [x] DataChannel 接收控制指令
- [x] ROS2 消息类型定义
- [x] ROS2 客户端框架
- [ ] ROS2 控制消息发布实现
- [ ] ROS2 视频源订阅
- [ ] 网页键盘控制
- [ ] 录像功能

## 🎯 下一步计划

### 优先级 1：ROS2 完整集成

1. **完成控制消息发布**
   - 实现 `publishControlToROS2` 函数
   - 使用 rclgo 或命令行工具
   - 测试控制指令传输

2. **ROS2 视频源支持**
   - 订阅 `/camera_front_wide`
   - ROS2 Image → GStreamer 转换
   - 与现有 H.264 编码集成

### 优先级 2：用户体验提升

3. **网页键盘控制** - WASD 控制移动，QE 控制臂架
4. **游戏手柄支持** - 使用 Gamepad API
5. **录像功能** - MediaRecorder 录制视频

### 优先级 3：性能优化

6. **零拷贝优化** - 减少内存拷贝
7. **自适应码率** - 根据网络状况调整
8. **多摄像头支持** - 同时传输多路视频

## 📄 许可证

MIT License

