package main

import (
    "bufio"
    "encoding/binary"
    "encoding/json"
    "flag"
    "fmt"
    "io"
    "log"
    "net"
    "os"
    "os/exec"
    "path/filepath"
    "sync"
    "time"

	"github.com/gorilla/websocket"
	"github.com/pion/webrtc/v4"
	"github.com/pion/webrtc/v4/pkg/media"
	// "github.com/supabase-community/supabase-go"
)

type SignalingMessage struct {
	Type    string          `json:"type"`
	From    string          `json:"from"`
	To      string          `json:"to"`
	Payload json.RawMessage `json:"payload"`
}

var (
	signalingURL = flag.String("signaling", "ws://192.168.124.3:8090/ws", "信令服务器地址")
	// 视频源固定为 ROS2 Bridge，不再需要本地摄像头参数
	ros2ImageTopic   = flag.String("ros2-image-topic", "/stitched_image", "ROS2 视频话题")
	ros2ControlTopic = flag.String("ros2-control-topic", "/controls/teleop", "ROS2 控制话题")
	defaultFPS       = flag.Int("default-fps", 30, "在无法计算时间戳时的默认视频帧率")

	// 音频配置
	enableAudio       = flag.Bool("enable-audio", true, "是否启用音频采集")
	audioDevice       = flag.String("audio-device", "", "音频设备名称（空值使用系统默认）")
	audioBitrate      = flag.Int("audio-bitrate", 32000, "Opus 音频比特率 (bps)")
	audioUDPPort      = flag.Int("audio-udp-port", 5004, "音频采集 RTP UDP 端口")
	audioPlaybackPort = flag.Int("audio-playback-port", 5005, "音频播放 RTP UDP 端口")

	// Supabase 配置
	supabaseURL = flag.String("supabase-url", "", "Supabase URL")
	supabaseKey = flag.String("supabase-key", "", "Supabase Service Key")

	peerConnection *webrtc.PeerConnection
	// 保护对 WebSocket 连接的写操作，避免 concurrent write panic
	wsWriteMu sync.Mutex
)

var (
	videoTrack        *webrtc.TrackLocalStaticSample
	audioTrack        *webrtc.TrackLocalStaticRTP
	controlStdin      io.WriteCloser
	audioPlaybackConn net.Conn // UDP 连接，用于发送接收到的音频到播放器
	
	// 重连配置
	reconnectBaseDelay = 2 * time.Second
	reconnectMaxDelay  = 60 * time.Second
)

func main() {
	flag.Parse()

	log.SetFlags(log.Ltime | log.Lshortfile)

	log.Printf("🚀 挖掘机端启动 (支持自动重连)...")

	// 初始化 Supabase 并启动心跳
	// initSupabase()
	// startHeartbeat()

	// ====== 一次性初始化：轨道和子进程 ======
	var err error

	// 提前创建轨道 (只创建一次，可复用)
	videoTrack, err = webrtc.NewTrackLocalStaticSample(
		webrtc.RTPCodecCapability{MimeType: "video/h264"},
		"video",
		"excavator-video",
	)
	if err != nil {
		log.Fatalf("❌ 创建视频轨道失败: %v", err)
	}

	// 创建音频轨道 (使用 RTP 类型，因为 GStreamer 已经打包好 RTP)
	audioTrack, err = webrtc.NewTrackLocalStaticRTP(
		webrtc.RTPCodecCapability{MimeType: webrtc.MimeTypeOpus},
		"audio",
		"excavator-audio",
	)
	if err != nil {
		log.Fatalf("❌ 创建音频轨道失败: %v", err)
	}

	// 启动视频编码器 (SHM -> GStreamer -> stdout)
	go startVideoStreamForwarder(videoTrack)

	// 启动音频采集和转发 (USB Mic -> GStreamer -> UDP -> audioTrack)
	if *enableAudio {
		go startAudioStreamer(*audioDevice, *audioBitrate, *audioUDPPort)
		go startAudioStreamForwarder(audioTrack, *audioUDPPort)
		// 启动音频播放器 (接收来自控制端的语音)
		go startAudioPlayer(*audioPlaybackPort)
	}

	// 启动控制转发器 (stdin -> ROS2)
	go startControlStreamForwarder(*ros2ControlTopic)

	// ====== 无限重连循环 ======
	reconnectDelay := reconnectBaseDelay
	consecutiveFailures := 0

	for {
		log.Printf("📡 正在连接信令服务器: %s", *signalingURL)
		
		connected, err := connectAndServe(*signalingURL)
		
		if err != nil {
			log.Printf("❌ 连接断开: %v", err)
		}

		// 清理旧的 PeerConnection
		if peerConnection != nil {
			peerConnection.Close()
			peerConnection = nil
		}

		// 如果曾经成功连接过，重置退避延迟
		if connected {
			reconnectDelay = reconnectBaseDelay
			consecutiveFailures = 0
			log.Printf("🔄 连接已断开，立即重连...")
		} else {
			consecutiveFailures++
			log.Printf("🔄 连接失败 (第 %d 次)，%v 后重连...", consecutiveFailures, reconnectDelay)
			time.Sleep(reconnectDelay)

			// 指数退避，但不超过最大值
			reconnectDelay = reconnectDelay * 2
			if reconnectDelay > reconnectMaxDelay {
				reconnectDelay = reconnectMaxDelay
			}
		}
	}
}

// connectAndServe 连接到信令服务器并处理消息，直到连接断开
// 返回: (是否曾成功连接, 错误)
func connectAndServe(signalingURL string) (bool, error) {
	// 连接信令服务器
	conn, _, err := websocket.DefaultDialer.Dial(signalingURL, nil)
	if err != nil {
		return false, fmt.Errorf("连接信令服务器失败: %w", err)
	}
	defer conn.Close()

	// 注册为 excavator
	registerMsg := map[string]string{
		"type":     "register",
		"identity": "excavator",
	}
	wsWriteMu.Lock()
	err = conn.WriteJSON(registerMsg)
	wsWriteMu.Unlock()
	if err != nil {
		return false, fmt.Errorf("注册失败: %w", err)
	}
	log.Printf("✅ 已注册为 excavator")
	log.Printf("⏳ 等待控制端连接...")

	// 启动 WebSocket 心跳保活 (在独立 goroutine 中)
	stopHeartbeat := make(chan struct{})
	go func() {
		ticker := time.NewTicker(30 * time.Second)
		defer ticker.Stop()
		for {
			select {
			case <-stopHeartbeat:
				return
			case <-ticker.C:
				wsWriteMu.Lock()
				err := conn.WriteMessage(websocket.PingMessage, nil)
				wsWriteMu.Unlock()
				if err != nil {
					log.Printf("⚠️ 发送 WebSocket Ping 失败: %v", err)
					return
				}
			}
		}
	}()
	defer close(stopHeartbeat)

	// 处理信令消息 (阻塞，直到连接断开)
	// 返回 true 表示曾经成功连接过
	return true, handleSignaling(conn)
}

func handleSignaling(conn *websocket.Conn) error {
	for {
		var msg SignalingMessage
		if err := conn.ReadJSON(&msg); err != nil {
			if peerConnection != nil {
				peerConnection.Close()
				peerConnection = nil
			}
			return fmt.Errorf("读取信令消息失败: %w", err)
		}

		// log.Printf("📨 收到信令: %s (来自 %s)", msg.Type, msg.From) // 调试时取消注释

		switch msg.Type {
		case "offer":
			if peerConnection != nil {
				log.Printf("ℹ️ 收到新 Offer，关闭旧连接...")
				peerConnection.Close()
				peerConnection = nil
			}

			var err error
			peerConnection, err = createPeerConnection(conn)
			if err != nil {
				log.Printf("❌ 创建新 PeerConnection 失败: %v", err)
				continue
			}

			var offer webrtc.SessionDescription
			if err := json.Unmarshal(msg.Payload, &offer); err != nil {
				log.Printf("❌ 解析 Offer 失败: %v", err)
				continue
			}

			if err := peerConnection.SetRemoteDescription(offer); err != nil {
				log.Printf("❌ 设置 RemoteDescription 失败: %v", err)
				continue
			}

			answer, err := peerConnection.CreateAnswer(nil)
			if err != nil {
				log.Printf("❌ 创建 Answer 失败: %v", err)
				continue
			}

			if err := peerConnection.SetLocalDescription(answer); err != nil {
				log.Printf("❌ 设置 LocalDescription 失败: %v", err)
				continue
			}

			// Trickle ICE: 立即发送 Answer
			answerData, _ := json.Marshal(peerConnection.LocalDescription())
			answerMsg := SignalingMessage{
				Type:    "answer",
				From:    "excavator",
				To:      "controller",
				Payload: answerData,
			}

			wsWriteMu.Lock()
			err = conn.WriteJSON(answerMsg)
			wsWriteMu.Unlock()
			if err != nil {
				log.Printf("❌ 发送 Answer 失败: %v", err)
			} else {
				log.Printf("✅ Answer 已发送")
			}

		case "candidate":
			if peerConnection == nil {
				log.Printf("⚠️ 收到 ICE candidate，但 PeerConnection 未创建")
				continue
			}
			var candidate webrtc.ICECandidateInit
			if err := json.Unmarshal(msg.Payload, &candidate); err != nil {
				log.Printf("❌ 解析 ICE 候选失败: %v", err)
				continue
			}

			if err := peerConnection.AddICECandidate(candidate); err != nil {
				log.Printf("❌ 添加 ICE 候选失败: %v", err)
			}
		}
	}
}

func createPeerConnection(conn *websocket.Conn) (*webrtc.PeerConnection, error) {
	pc, err := webrtc.NewPeerConnection(webrtc.Configuration{})
	if err != nil {
		return nil, err
	}

	pc.OnICEConnectionStateChange(func(state webrtc.ICEConnectionState) {
		log.Printf("🔗 ICE 连接状态: %s", state.String())
		if state == webrtc.ICEConnectionStateFailed || state == webrtc.ICEConnectionStateClosed || state == webrtc.ICEConnectionStateDisconnected {
			log.Printf("🔴 ICE 连接已断开，清理资源...")
			if pc != nil {
				pc.Close()
				peerConnection = nil
			}
		}
	})

	pc.OnICECandidate(func(c *webrtc.ICECandidate) {
		if c == nil {
			return
		}

		candidateData, err := json.Marshal(c.ToJSON())
		if err != nil {
			log.Printf("❌ 序列化 ICE 候选失败: %v", err)
			return
		}

		candidateMsg := SignalingMessage{
			Type:    "candidate",
			From:    "excavator",
			To:      "controller",
			Payload: candidateData,
		}

		wsWriteMu.Lock()
		err = conn.WriteJSON(candidateMsg)
		wsWriteMu.Unlock()
		if err != nil {
			log.Printf("❌ 发送 ICE 候选失败: %v", err)
		}
	})

	pc.OnDataChannel(func(dc *webrtc.DataChannel) {
		log.Printf("✅ 浏览器创建了 DataChannel: '%s'", dc.Label())

		dc.OnOpen(func() {
			log.Printf("✅ DataChannel '%s' 已打开", dc.Label())
		})

        dc.OnMessage(func(msg webrtc.DataChannelMessage) {
            if controlStdin != nil {
                go publishControlToROS2(msg.Data)
            } else {
                log.Printf("⚠️ 收到控制指令，但控制管道未就绪")
            }
        })

		dc.OnClose(func() {
			log.Printf("🔴 DataChannel '%s' 已关闭", dc.Label())
		})
	})

	// 处理接收到的远程音频轨道 (来自控制端的麦克风)
	pc.OnTrack(func(track *webrtc.TrackRemote, receiver *webrtc.RTPReceiver) {
		log.Printf("🎧 收到远程轨道: %s (类型: %s)", track.ID(), track.Kind().String())

		if track.Kind() == webrtc.RTPCodecTypeAudio {
			log.Printf("🔊 开始接收控制端音频...")
			go forwardRemoteAudioToPlayer(track)
		}
	})

	if _, err = pc.AddTrack(videoTrack); err != nil {
		return nil, fmt.Errorf("❌ 添加视频轨道失败: %w", err)
	}
	if _, err = pc.AddTrack(audioTrack); err != nil {
		return nil, fmt.Errorf("❌ 添加音频轨道失败: %w", err)
	}

	return pc, nil
}

func startVideoStreamForwarder(videoTrack *webrtc.TrackLocalStaticSample) {
    log.Printf("🚀 启动 GStreamer 硬件编码器 (SHM -> stdout)...")

    exePath, err := os.Executable()
    if err != nil {
        log.Fatalf("❌ 无法获取当前执行路径: %v", err)
    }
    scriptPath := filepath.Join(filepath.Dir(exePath), "..", "scripts", "shm_solution", "shm_to_stdout.py")

    cmd := exec.Command("python3", scriptPath)
    stdout, err := cmd.StdoutPipe()
    if err != nil {
        log.Fatalf("❌ [Video] 创建 stdout 管道失败: %v", err)
    }
    cmd.Stderr = os.Stderr
    if err := cmd.Start(); err != nil {
        log.Fatalf("❌ [Video] 启动 shm_to_stdout.py 失败: %v", err)
    }
    log.Printf("✅ GStreamer 编码器已启动 (PID: %d)", cmd.Process.Pid)

    go func() {
        defer cmd.Process.Kill()
        defer stdout.Close()

        reader := bufio.NewReaderSize(stdout, 128*1024)
        header := make([]byte, 12) // 4B length + 8B timestamp_ns (big-endian)
        var lastTs uint64

        for {
            if _, err := io.ReadFull(reader, header); err != nil {
                if err == io.EOF {
                    log.Printf("ℹ️ [Video] stdout 结束")
                } else {
                    log.Printf("❌ [Video] 读取头失败: %v", err)
                }
                break
            }
            frameLen := binary.BigEndian.Uint32(header[0:4])
            tsNs := binary.BigEndian.Uint64(header[4:12])
            if frameLen == 0 || frameLen > 2*1024*1024 {
                log.Printf("⚠️ [Video] 异常帧长: %d", frameLen)
                continue
            }
            frame := make([]byte, frameLen)
            if _, err := io.ReadFull(reader, frame); err != nil {
                log.Printf("❌ [Video] 读取帧失败: %v", err)
                break
            }
            dur := time.Second / time.Duration(*defaultFPS)
            if lastTs > 0 && tsNs > lastTs {
                dur = time.Duration(tsNs - lastTs)
            }
            lastTs = tsNs
            _ = videoTrack.WriteSample(media.Sample{Data: frame, Duration: dur})
        }
    }()
}

func startControlStreamForwarder(controlTopic string) {
    log.Printf("🚀 启动 ROS2 控制接收器 (stdin -> ROS2)...")

    rosPath, rosPathOk := os.LookupEnv("ROS_DISTRO")
    if !rosPathOk || rosPath == "" {
        log.Fatalf("❌ ROS2 环境未加载 (ROS_DISTRO 未设置). 请先 source /opt/ros/humble/setup.bash")
    }

    exePath, err := os.Executable()
    if err != nil {
        log.Fatalf("❌ 无法获取当前执行路径: %v", err)
    }
    scriptPath := filepath.Join(filepath.Dir(exePath), "..", "scripts", "shm_solution", "ros_control_stdin.py")

    cmd := exec.Command("python3", scriptPath, "--control-topic", controlTopic)
    controlStdin, err = cmd.StdinPipe()
    if err != nil {
        log.Fatalf("❌ [Control] 创建 stdin 管道失败: %v", err)
    }
    cmd.Stderr = os.Stderr
    if err := cmd.Start(); err != nil {
        log.Fatalf("❌ [Control] 启动 ros_control_stdin.py 失败: %v", err)
    }
    log.Printf("✅ ROS2 控制器已启动 (PID: %d)", cmd.Process.Pid)
}

func publishControlToROS2(data []byte) {
    if controlStdin == nil {
		log.Printf("⚠️ 控制管道 (stdin) 未就绪")
		return
	}
	msg := append(data, '\n')
    _, err := controlStdin.Write(msg)
	if err != nil {
		log.Printf("❌ 写入控制指令到 stdin 失败: %v", err)
		// 如果写入失败，可能是 Python 进程已退出，这里可以记录但不中断程序
	}
}

// --- Supabase Integration ---

// const DeviceSN = "1421323042255"

// func initSupabase() {
// 	if *supabaseURL == "" || *supabaseKey == "" {
// 		log.Println("⚠️ 未提供 Supabase URL 或 Key，跳过 Supabase 初始化。")
// 		return
// 	}

// 	var err error
// 	// 使用 supabase-community/supabase-go 初始化客户端
// 	supabaseClient, err = supabase.NewClient(*supabaseURL, *supabaseKey, nil)
// 	if err != nil {
// 		log.Fatalf("❌ 初始化 Supabase 客户端失败: %v", err)
// 	}
// 	log.Println("✅ Supabase 客户端初始化成功")
// }

// func startHeartbeat() {
// 	if supabaseClient == nil {
// 		return
// 	}

// 	// 1. 上线状态更新
// 	log.Printf("🔄 正在更新设备状态为 online (SN: %s)...", DeviceSN)
	
// 	// 构造更新数据
// 	// 注意：Supabase Go 库的 Update 方法签名可能因版本而异，这里假设遵循 postgrest-go 风格
// 	payload := map[string]interface{}{
// 		"status":     "online",
// 		"ip_address": "192.168.3.57", // 真实 IP
// 		"last_seen":  time.Now().Format(time.RFC3339),
// 	}

// 	// 执行更新: UPDATE excavators SET ... WHERE serial_sn = DeviceSN
// 	// Update(data, count, returnRepresentation)
// 	_, _, err := supabaseClient.From("excavators").Update(payload, "", "").Eq("serial_sn", DeviceSN).Execute()
// 	if err != nil {
// 		log.Printf("❌ 更新上线状态失败: %v (请检查表 excavators 是否存在且包含 serial_sn=%s)", err, DeviceSN)
// 	} else {
// 		log.Println("✅ 设备状态已更新为 Online")
// 	}

// 	// 2. 开启心跳循环 (每 10 秒)
// 	go func() {
// 		ticker := time.NewTicker(10 * time.Second)
// 		defer ticker.Stop()

// 		for range ticker.C {
// 			hbPayload := map[string]interface{}{
// 				"last_seen": time.Now().Format(time.RFC3339),
// 			}
// 			_, _, err := supabaseClient.From("excavators").Update(hbPayload, "", "").Eq("serial_sn", DeviceSN).Execute()
// 			if err != nil {
// 				log.Printf("❌ 心跳发送失败: %v", err)
// 			} else {
// 				// log.Printf("💓 心跳发送成功") // 减少日志噪音，可选开启
// 			}
// 		}
// 	}()
// }

// --- Audio Streaming Functions ---

func startAudioStreamer(deviceName string, bitrate int, udpPort int) {
    log.Printf("🎤 启动音频采集器 (USB Mic -> GStreamer -> UDP)...")

    exePath, err := os.Executable()
    if err != nil {
        log.Fatalf("❌ 无法获取当前执行路径: %v", err)
    }
    scriptPath := filepath.Join(filepath.Dir(exePath), "..", "scripts", "shm_solution", "audio_streamer.py")

    args := []string{scriptPath}
    if deviceName != "" {
        args = append(args, "--device", deviceName)
    }
    args = append(args, "--bitrate", fmt.Sprintf("%d", bitrate))
    args = append(args, "--port", fmt.Sprintf("%d", udpPort))

    cmd := exec.Command("python3", args...)
    cmd.Stderr = os.Stderr
    cmd.Stdout = os.Stdout

    if err := cmd.Start(); err != nil {
        log.Printf("❌ [Audio] 启动 audio_streamer.py 失败: %v", err)
        return
    }
    log.Printf("✅ 音频采集器已启动 (PID: %d)", cmd.Process.Pid)

    // 监控进程
    go func() {
        if err := cmd.Wait(); err != nil {
            log.Printf("⚠️ [Audio] audio_streamer.py 退出: %v", err)
        }
    }()
}

func startAudioStreamForwarder(audioTrack *webrtc.TrackLocalStaticRTP, udpPort int) {
    log.Printf("🔊 启动音频 UDP 接收器 (端口: %d)...", udpPort)

    // 监听本地 UDP 端口
    addr := fmt.Sprintf("127.0.0.1:%d", udpPort)
    listener, err := net.ListenPacket("udp", addr)
    if err != nil {
        log.Printf("❌ [Audio] UDP 监听失败: %v", err)
        return
    }
    defer listener.Close()

    log.Printf("✅ 音频 UDP 接收器已启动")

    buffer := make([]byte, 1500) // MTU size usually < 1500

    for {
        n, _, err := listener.ReadFrom(buffer)
        if err != nil {
            log.Printf("❌ [Audio] UDP 读取错误: %v", err)
            continue
        }

        // 直接把收到的 RTP 包写给 WebRTC Audio Track
        if _, err := audioTrack.Write(buffer[:n]); err != nil {
            if err == io.EOF {
                log.Printf("ℹ️ [Audio] Audio Track 已关闭")
                return
            }
            // log.Printf("⚠️ [Audio] Track 写入错误: %v", err) // 可能会很嘈杂，按需取消注释
        }
	}
}

// --- Audio Playback Functions (接收控制端语音) ---

func startAudioPlayer(udpPort int) {
    log.Printf("🔈 启动音频播放器 (UDP -> GStreamer -> 扬声器)...")

    exePath, err := os.Executable()
    if err != nil {
        log.Printf("❌ 无法获取当前执行路径: %v", err)
        return
    }
    scriptPath := filepath.Join(filepath.Dir(exePath), "..", "scripts", "shm_solution", "audio_player.py")

    cmd := exec.Command("python3", scriptPath, "--port", fmt.Sprintf("%d", udpPort))
    cmd.Stderr = os.Stderr
    cmd.Stdout = os.Stdout

    if err := cmd.Start(); err != nil {
        log.Printf("❌ [AudioPlayer] 启动 audio_player.py 失败: %v", err)
        return
    }
    log.Printf("✅ 音频播放器已启动 (PID: %d)", cmd.Process.Pid)

    // 建立 UDP 连接用于发送音频数据
    addr := fmt.Sprintf("127.0.0.1:%d", udpPort)
    conn, err := net.Dial("udp", addr)
    if err != nil {
        log.Printf("❌ [AudioPlayer] UDP 连接失败: %v", err)
        return
    }
    audioPlaybackConn = conn
    log.Printf("✅ 音频播放 UDP 连接已建立 (目标: %s)", addr)

    // 监控进程
    go func() {
        if err := cmd.Wait(); err != nil {
            log.Printf("⚠️ [AudioPlayer] audio_player.py 退出: %v", err)
        }
        if audioPlaybackConn != nil {
            audioPlaybackConn.Close()
            audioPlaybackConn = nil
        }
    }()
}

func forwardRemoteAudioToPlayer(track *webrtc.TrackRemote) {
    buffer := make([]byte, 1500)

    for {
        n, _, err := track.Read(buffer)
        if err != nil {
            if err == io.EOF {
                log.Printf("ℹ️ [AudioPlayer] 远程音频轨道已关闭")
            } else {
                log.Printf("❌ [AudioPlayer] 读取远程音频失败: %v", err)
            }
            return
        }

        if audioPlaybackConn != nil {
            if _, err := audioPlaybackConn.Write(buffer[:n]); err != nil {
                log.Printf("⚠️ [AudioPlayer] UDP 发送失败: %v", err)
            }
        }
    }
}
