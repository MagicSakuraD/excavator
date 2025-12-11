# USB 麦克风音频采集实现计划

## 📋 可行性分析

### ✅ 当前状态
- `audioTrack` 已创建（`audio/opus`），类型正确
- `audioTrack` 已添加到 PeerConnection（第293行）
- **问题**：没有实际数据源，音频轨道为空

### ✅ 技术可行性
1. **WebRTC 支持**：✅ 已支持 `audio/opus` 轨道
2. **GStreamer 支持**：✅ 可以捕获 USB 麦克风（`pulsesrc`/`alsasrc`）
3. **Opus 编码**：✅ GStreamer 有 `opusenc` 插件
4. **架构一致性**：✅ 与视频流架构完全一致（GStreamer → stdout → Go）

### 🎯 推荐方案：GStreamer + Python（与视频流一致）

**优点**：
- 与现有视频流架构完全一致
- Jetson Orin 上 GStreamer 已优化
- 进程隔离，避免库冲突
- 可复用现有的进程管理机制

---

## 📐 实现计划

### 阶段 1：创建音频采集脚本

**文件**：`scripts/shm_solution/mic_to_stdout.py`

**功能**：
- 使用 GStreamer 捕获 USB 麦克风音频
- 编码为 Opus 格式
- 输出到 stdout（格式：12字节头部 + Opus 数据包）

**GStreamer 管道**：
```python
pulsesrc device=<device_name> ! 
audioconvert ! 
audioresample ! 
opusenc bitrate=64000 ! 
appsink
```

**输出格式**（与视频流一致）：
- 12字节头部：`[4字节长度][8字节时间戳(纳秒)]`
- 数据：Opus 编码的音频包

**参数**：
- `--device`: 音频设备名称（可选，默认使用系统默认设备）
- `--sample-rate`: 采样率（默认 48000 Hz，WebRTC 标准）
- `--bitrate`: Opus 比特率（默认 64 kbps）

---

### 阶段 2：实现 Go 音频转发器

**文件**：`cmd/excavator/main.go`

**新增函数**：`startAudioStreamForwarder(audioTrack *webrtc.TrackLocalStaticSample)`

**功能**：
1. 启动 `mic_to_stdout.py` 子进程
2. 从 stdout 读取音频数据（格式与视频流相同）
3. 解析 12 字节头部（长度 + 时间戳）
4. 读取 Opus 数据包
5. 写入 `audioTrack`（使用正确的时间戳）

**关键代码结构**：
```go
func startAudioStreamForwarder(audioTrack *webrtc.TrackLocalStaticSample) {
    // 1. 启动 Python 脚本
    cmd := exec.Command("python3", scriptPath, "--device", deviceName)
    stdout, _ := cmd.StdoutPipe()
    
    // 2. 读取循环（与视频流类似）
    reader := bufio.NewReaderSize(stdout, 32*1024)
    header := make([]byte, 12)
    var lastTs uint64
    
    for {
        // 读取头部
        io.ReadFull(reader, header)
        frameLen := binary.BigEndian.Uint32(header[0:4])
        tsNs := binary.BigEndian.Uint64(header[4:12])
        
        // 读取 Opus 数据包
        frame := make([]byte, frameLen)
        io.ReadFull(reader, frame)
        
        // 计算持续时间
        dur := 20 * time.Millisecond // Opus 默认帧长
        if lastTs > 0 && tsNs > lastTs {
            dur = time.Duration(tsNs - lastTs)
        }
        lastTs = tsNs
        
        // 写入 WebRTC 轨道
        audioTrack.WriteSample(media.Sample{
            Data: frame,
            Duration: dur,
        })
    }
}
```

---

### 阶段 3：添加配置参数

**在 `main.go` 中添加命令行参数**：
```go
var (
    // ... 现有参数 ...
    audioDevice   = flag.String("audio-device", "", "音频设备名称（空值使用系统默认）")
    audioBitrate  = flag.Int("audio-bitrate", 64, "Opus 音频比特率 (kbps)")
    enableAudio   = flag.Bool("enable-audio", true, "是否启用音频采集")
)
```

---

### 阶段 4：集成到主流程

**在 `main()` 函数中**：
```go
// 启动视频编码器
go startVideoStreamForwarder(videoTrack)

// 启动音频采集（如果启用）
if *enableAudio {
    go startAudioStreamForwarder(audioTrack)
}
```

---

## 🔧 技术细节

### 1. 音频设备选择

**方案 A：使用 PulseAudio（推荐）**
- `pulsesrc device=<name>`：指定设备名称
- 优点：自动处理设备切换，支持 USB 热插拔
- 获取设备列表：`pactl list sources short`

**方案 B：使用 ALSA**
- `alsasrc device=hw:X,Y`：直接访问硬件
- 优点：延迟更低
- 缺点：需要手动指定设备编号

**推荐**：优先使用 PulseAudio，如果延迟要求极高，可切换到 ALSA。

### 2. Opus 编码参数

**标准配置**：
- **采样率**：48000 Hz（WebRTC 标准）
- **声道**：单声道（mono）或立体声（stereo）
- **比特率**：32-128 kbps（推荐 64 kbps）
- **帧长**：20 ms（WebRTC 标准）

**GStreamer 管道示例**：
```python
pulsesrc device="<device>" ! 
audioconvert ! 
audioresample ! 
audio/x-raw,rate=48000,channels=1,format=S16LE ! 
opusenc bitrate=64000 frame-size=20 ! 
appsink
```

### 3. 时间戳处理

- **视频流**：使用共享内存中的时间戳（纳秒精度）
- **音频流**：使用 GStreamer 的时间戳（基于采样率计算）
- **同步**：音频和视频使用独立的时间戳，WebRTC 会自动处理同步

---

## 📝 文件清单

### 新增文件
1. `scripts/shm_solution/mic_to_stdout.py` - 音频采集脚本

### 修改文件
1. `cmd/excavator/main.go` - 添加音频转发器函数和配置参数

### 可选文件
1. `scripts/shm_solution/start_audio.sh` - 独立测试脚本（可选）

---

## ✅ 测试步骤

1. **测试音频设备**：
   ```bash
   # 列出可用音频设备
   pactl list sources short
   
   # 测试 GStreamer 管道
   gst-launch-1.0 pulsesrc device="<device>" ! audioconvert ! opusenc ! fakesink
   ```

2. **测试 Python 脚本**：
   ```bash
   python3 scripts/shm_solution/mic_to_stdout.py --device "<device>" | hexdump -C | head
   ```

3. **集成测试**：
   ```bash
   # 启动完整系统（包含音频）
   ./start_all.sh ws://192.168.0.87:8090/ws
   
   # 在浏览器中检查音频是否正常
   ```

---

## 🚨 潜在问题与解决方案

### 问题 1：找不到 USB 麦克风设备
**解决**：
- 检查设备是否已连接：`lsusb`
- 检查 PulseAudio 是否识别：`pactl list sources`
- 可能需要设置环境变量：`PULSE_SERVER=...`

### 问题 2：音频延迟过高
**解决**：
- 使用 ALSA 直接访问硬件（绕过 PulseAudio）
- 减少 Opus 帧长（但会增加 CPU 占用）
- 优化 GStreamer 管道缓冲区

### 问题 3：音频与视频不同步
**解决**：
- WebRTC 会自动处理同步（基于时间戳）
- 确保音频和视频使用相同的时间基准
- 如果仍有问题，检查网络延迟

---

## 📊 预期效果

- ✅ USB 麦克风音频实时采集
- ✅ Opus 编码，低延迟（~20ms）
- ✅ 通过 WebRTC 传输到浏览器
- ✅ 与视频流同步播放
- ✅ 架构与视频流一致，易于维护

---

## 🎯 下一步行动

1. 创建 `mic_to_stdout.py` 脚本
2. 在 `main.go` 中实现 `startAudioStreamForwarder`
3. 添加命令行参数支持
4. 测试音频采集和传输
5. 更新 `README.md` 文档



