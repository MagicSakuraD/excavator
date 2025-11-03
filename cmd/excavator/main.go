package main

import (
	"bufio"
	"encoding/binary"
	"encoding/json"
	"flag"
	"fmt"
	"io"
	"log"
	"os"
	"os/exec"
	"path/filepath"
	"time"

	"github.com/go-gst/go-gst/gst"
	"github.com/go-gst/go-gst/gst/app"
	"github.com/gorilla/websocket"
	"github.com/pion/webrtc/v4"
	"github.com/pion/webrtc/v4/pkg/media"
)

type SignalingMessage struct {
	Type    string          `json:"type"`
	From    string          `json:"from"`
	To      string          `json:"to"`
	Payload json.RawMessage `json:"payload"`
}

var (
	signalingURL = flag.String("signaling", "ws://localhost:8090/ws", "信令服务器地址")
	// 视频源固定为 ROS2 Bridge，不再需要本地摄像头参数
	ros2ImageTopic   = flag.String("ros2-image-topic", "/camera_front_wide", "ROS2 视频话题")
	ros2ControlTopic = flag.String("ros2-control-topic", "/controls/teleop", "ROS2 控制话题")
	defaultFPS       = flag.Int("default-fps", 30, "在无法计算时间戳时的默认视频帧率")

	peerConnection *webrtc.PeerConnection
)

var (
	videoTrack  *webrtc.TrackLocalStaticSample
	audioTrack  *webrtc.TrackLocalStaticSample
	pythonStdin io.WriteCloser
)

func main() {
	flag.Parse()

	log.SetFlags(log.Ltime | log.Lshortfile)

	// 初始化 GStreamer
	gst.Init(nil)

	log.Printf("🚀 挖掘机端启动 (精简版)...")
	log.Printf("📡 连接信令服务器: %s", *signalingURL)

	// 连接信令服务器
	conn, _, err := websocket.DefaultDialer.Dial(*signalingURL, nil)
	if err != nil {
		log.Fatalf("❌ 连接信令服务器失败: %v", err)
	}
	defer conn.Close()

	// 注册为 excavator
	registerMsg := map[string]string{
		"type":     "register",
		"identity": "excavator",
	}
	if err := conn.WriteJSON(registerMsg); err != nil {
		log.Fatalf("❌ 注册失败: %v", err)
	}
	log.Printf("✅ 已注册为 excavator")

	// 提前创建轨道
	videoTrack, err = webrtc.NewTrackLocalStaticSample(
		webrtc.RTPCodecCapability{MimeType: "video/h264"},
		"video",
		"excavator-video",
	)
	if err != nil {
		log.Fatalf("❌ 创建视频轨道失败: %v", err)
	}

	audioTrack, err = webrtc.NewTrackLocalStaticSample(
		webrtc.RTPCodecCapability{MimeType: "audio/opus"},
		"audio",
		"excavator-audio",
	)
	if err != nil {
		log.Fatalf("❌ 创建音频轨道失败: %v", err)
	}

	// 启动 Python 双向桥接 (视频输入/控制输出)
	go startH264BridgeForwarder(*ros2ImageTopic, *ros2ControlTopic, videoTrack)
	// 启动 GStreamer 音频管道 (测试音源)
	go startGStreamerPipeline("opus", []*webrtc.TrackLocalStaticSample{audioTrack}, "audiotestsrc")

	log.Printf("⏳ 等待控制端连接...")

	// 处理信令消息 (阻塞)
	handleSignaling(conn)

	// 保持运行
	select {}
}

func handleSignaling(conn *websocket.Conn) {
	for {
		var msg SignalingMessage
		if err := conn.ReadJSON(&msg); err != nil {
			log.Printf("❌ 读取信令消息失败: %v", err)
			if peerConnection != nil {
				peerConnection.Close()
				peerConnection = nil
			}
			return
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

			if err := conn.WriteJSON(answerMsg); err != nil {
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

		if err := conn.WriteJSON(candidateMsg); err != nil {
			log.Printf("❌ 发送 ICE 候选失败: %v", err)
		}
	})

	pc.OnDataChannel(func(dc *webrtc.DataChannel) {
		log.Printf("✅ 浏览器创建了 DataChannel: '%s'", dc.Label())

		dc.OnOpen(func() {
			log.Printf("✅ DataChannel '%s' 已打开", dc.Label())
		})

		dc.OnMessage(func(msg webrtc.DataChannelMessage) {
			if pythonStdin != nil {
				go publishControlToROS2(msg.Data)
			}
		})

		dc.OnClose(func() {
			log.Printf("🔴 DataChannel '%s' 已关闭", dc.Label())
		})
	})

	if _, err = pc.AddTrack(videoTrack); err != nil {
		return nil, fmt.Errorf("❌ 添加视频轨道失败: %w", err)
	}
	if _, err = pc.AddTrack(audioTrack); err != nil {
		return nil, fmt.Errorf("❌ 添加音频轨道失败: %w", err)
	}

	return pc, nil
}

func startGStreamerPipeline(codecName string, tracks []*webrtc.TrackLocalStaticSample, pipelineSrc string) {
	pipelineStr := "appsink name=appsink"

	switch codecName {
	case "opus":
		pipelineStr = pipelineSrc + " ! opusenc ! " + pipelineStr
	default:
		log.Fatalf("不支持的音频编解码器: %s", codecName)
	}

	log.Printf("🎬 启动 GStreamer 管道: %s", codecName)

	pipeline, err := gst.NewPipelineFromString(pipelineStr)
	if err != nil {
		log.Fatalf("❌ 创建 GStreamer 管道失败: %v", err)
	}

	if err = pipeline.SetState(gst.StatePlaying); err != nil {
		log.Fatalf("❌ 启动 GStreamer 管道失败: %v", err)
	}

	appSink, err := pipeline.GetElementByName("appsink")
	if err != nil {
		log.Fatalf("❌ 获取 appsink 失败: %v", err)
	}

	app.SinkFromElement(appSink).SetCallbacks(&app.SinkCallbacks{
		NewSampleFunc: func(sink *app.Sink) gst.FlowReturn {
			sample := sink.PullSample()
			if sample == nil {
				return gst.FlowEOS
			}
			buffer := sample.GetBuffer()
			if buffer == nil {
				return gst.FlowError
			}
			samples := buffer.Map(gst.MapRead).Bytes()
			defer buffer.Unmap()

			for _, t := range tracks {
				if err := t.WriteSample(media.Sample{
					Data:     samples,
					Duration: *buffer.Duration().AsDuration(),
				}); err != nil {
					// 忽略错误
				}
			}
			return gst.FlowOK
		},
	})
	log.Printf("✅ %s 管道运行中", codecName)
}

func startH264BridgeForwarder(videoTopic, controlTopic string, videoTrack *webrtc.TrackLocalStaticSample) {
	log.Printf("🚀 启动 Python 双向桥接...")

	rosPath, rosPathOk := os.LookupEnv("ROS_DISTRO")
	if !rosPathOk || rosPath == "" {
		log.Fatalf("❌ ROS2 环境未加载 (ROS_DISTRO 未设置). 请先 source /opt/ros/humble/setup.bash")
	}

	exePath, err := os.Executable()
	if err != nil {
		log.Fatalf("❌ 无法获取当前执行路径: %v", err)
	}
	scriptPath := filepath.Join(filepath.Dir(exePath), "..", "scripts", "ros2_h264_stdout_bridge.py")

	cmd := exec.Command("python3", scriptPath,
		"--video-topic", videoTopic,
		"--control-topic", controlTopic,
	)

	stdout, err := cmd.StdoutPipe()
	if err != nil {
		log.Fatalf("❌ 创建 stdout 管道失败: %v", err)
	}
	pythonStdin, err = cmd.StdinPipe()
	if err != nil {
		log.Fatalf("❌ 创建 stdin 管道失败: %v", err)
	}
	cmd.Stderr = os.Stderr

	if err := cmd.Start(); err != nil {
		log.Fatalf("❌ 启动 Python 桥接失败: %v", err)
	}
	log.Printf("✅ Python 桥接已启动 (PID: %d)", cmd.Process.Pid)

	go func() {
		defer cmd.Process.Kill()
		defer stdout.Close()
		defer pythonStdin.Close()

		reader := bufio.NewReader(stdout)
		headerBuf := make([]byte, 12) // 4字节长度 + 8字节时间戳
		var lastTimestampNs uint64

		for {
			_, err := io.ReadFull(reader, headerBuf)
			if err != nil {
				if err == io.EOF {
					log.Printf("ℹ️ Python 桥接 stdout 关闭，退出读取循环。")
				} else {
					log.Printf("❌ 读取帧头部失败: %v", err)
				}
				break
			}

			frameLength := binary.BigEndian.Uint32(headerBuf[0:4])
			timestampNs := binary.BigEndian.Uint64(headerBuf[4:12])

			if frameLength > 2*1024*1024 { // 2MB sanity check
				log.Printf("⚠️ 异常帧长度: %d, 跳过", frameLength)
				continue
			}

			frameData := make([]byte, frameLength)
			_, err = io.ReadFull(reader, frameData)
			if err != nil {
				log.Printf("❌ 读取帧数据失败: %v", err)
				break
			}

			var duration time.Duration
			if lastTimestampNs > 0 && timestampNs > lastTimestampNs {
				duration = time.Duration(timestampNs - lastTimestampNs)
			} else {
				duration = time.Second / time.Duration(*defaultFPS)
			}
			lastTimestampNs = timestampNs

			if err := videoTrack.WriteSample(media.Sample{Data: frameData, Duration: duration}); err != nil {
				// 忽略错误
			}
		}
	}()
}

func publishControlToROS2(data []byte) {
	if pythonStdin == nil {
		log.Printf("⚠️ 控制管道 (stdin) 未就绪")
		return
	}
	msg := append(data, '\n')
	_, err := pythonStdin.Write(msg)
	if err != nil {
		log.Printf("❌ 写入控制指令到 stdin 失败: %v", err)
	}
}
