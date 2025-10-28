package main

import (
	"encoding/json"
	"flag"
	"log"
	"net/http"
	"sync"

	"github.com/gorilla/websocket"
)

// Message 定义信令消息格式
type Message struct {
	Type    string          `json:"type"`    // "offer", "answer", "candidate"
	From    string          `json:"from"`    // "excavator" 或 "controller"
	To      string          `json:"to"`      // "excavator" 或 "controller"
	Payload json.RawMessage `json:"payload"` // SDP 或 ICE 候选数据
}

var (
	upgrader = websocket.Upgrader{
		CheckOrigin: func(r *http.Request) bool {
			return true // 允许所有来源（生产环境应该限制）
		},
	}
	
	clients = make(map[string]*websocket.Conn)
	mutex   sync.RWMutex
)

func handleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("❌ WebSocket 升级失败: %v", err)
		return
	}
	defer conn.Close()

	// 读取第一条消息以确定客户端身份
	var identityMsg struct {
		Type     string `json:"type"`
		Identity string `json:"identity"` // "excavator" 或 "controller"
	}

	if err := conn.ReadJSON(&identityMsg); err != nil {
		log.Printf("❌ 读取身份消息失败: %v", err)
		return
	}

	if identityMsg.Type != "register" {
		log.Printf("❌ 期望 'register' 消息，收到: %s", identityMsg.Type)
		return
	}

	identity := identityMsg.Identity
	log.Printf("✅ 客户端注册: %s (来自 %s)", identity, r.RemoteAddr)

	// 注册客户端
	mutex.Lock()
	clients[identity] = conn
	mutex.Unlock()

	defer func() {
		mutex.Lock()
		delete(clients, identity)
		mutex.Unlock()
		log.Printf("🔌 客户端断开: %s", identity)
	}()

	// 持续读取并转发消息
	for {
		var msg Message
		if err := conn.ReadJSON(&msg); err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("❌ WebSocket 错误: %v", err)
			}
			break
		}

		log.Printf("📨 收到消息: %s -> %s (类型: %s)", msg.From, msg.To, msg.Type)

		// 转发消息给目标客户端
		mutex.RLock()
		targetConn, exists := clients[msg.To]
		mutex.RUnlock()

		if exists {
			if err := targetConn.WriteJSON(msg); err != nil {
				log.Printf("❌ 转发消息失败: %v", err)
			} else {
				log.Printf("✅ 消息已转发")
			}
		} else {
			log.Printf("⚠️  目标客户端不在线: %s", msg.To)
		}
	}
}

func handleStatus(w http.ResponseWriter, r *http.Request) {
	mutex.RLock()
	defer mutex.RUnlock()

	status := struct {
		OnlineClients []string `json:"online_clients"`
		Count         int      `json:"count"`
	}{
		OnlineClients: make([]string, 0, len(clients)),
		Count:         len(clients),
	}

	for identity := range clients {
		status.OnlineClients = append(status.OnlineClients, identity)
	}

	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(status)
}

func main() {
	addr := flag.String("addr", ":8090", "信令服务器地址")
	flag.Parse()

	http.HandleFunc("/ws", handleWebSocket)
	http.HandleFunc("/status", handleStatus)

	log.Printf("🚀 信令服务器启动: %s", *addr)
	log.Printf("📡 WebSocket 端点: ws://<服务器IP>%s/ws", *addr)
	log.Printf("📊 状态查询: http://<服务器IP>%s/status", *addr)

	if err := http.ListenAndServe(*addr, nil); err != nil {
		log.Fatalf("❌ 服务器启动失败: %v", err)
	}
}

