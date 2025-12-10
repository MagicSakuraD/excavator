package main

import (
	"encoding/json"
	"flag"
	"log"
	"net/http"
	"sync"
	"time"

	"github.com/gorilla/websocket"
)

// 常量定义
const (
	// 写超时时间
	writeWait = 10 * time.Second
	// 读超时时间 (必须大于 pingPeriod)
	pongWait = 60 * time.Second
	// 心跳发送间隔 (必须小于 pongWait)
	pingPeriod = (pongWait * 9) / 10
)

// Message 表示信令消息
type Message struct {
	Type    string          `json:"type"`    // "offer" | "answer" | "candidate"
	From    string          `json:"from"`    // "excavator" | "controller"
	To      string          `json:"to"`      // "excavator" | "controller"
	Payload json.RawMessage `json:"payload"` // SDP 或 ICE 数据
}

// Client 封装 WebSocket 连接，提供写锁
type Client struct {
	Conn       *websocket.Conn
	Mu         sync.Mutex
	Identity   string
	RemoteAddr string
}

var (
	upgrader = websocket.Upgrader{
		CheckOrigin: func(r *http.Request) bool {
			// 内网测试场景，放开跨域；生产环境请收紧
			return true
		},
	}

	clientsMu sync.RWMutex
	clients   = make(map[string]*Client) // identity -> client
)

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

func handleWebSocket(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		log.Printf("❌ WebSocket 升级失败: %v", err)
		return
	}

	// 1. 设置读参数
	conn.SetReadLimit(1 << 20) // 1MB

	// 初始化读超时
	_ = conn.SetReadDeadline(time.Now().Add(pongWait))

	// 收到 Pong (客户端回应) 时，重置读超时
	conn.SetPongHandler(func(string) error {
		_ = conn.SetReadDeadline(time.Now().Add(pongWait))
		return nil
	})

	// 首条消息必须是 register
	var reg struct {
		Type     string `json:"type"`
		Identity string `json:"identity"`
	}
	if err := conn.ReadJSON(&reg); err != nil {
		log.Printf("❌ 读取身份消息失败: %v", err)
		conn.Close()
		return
	}
	if reg.Type != "register" || reg.Identity == "" {
		log.Printf("❌ 非法注册消息: %+v", reg)
		conn.Close()
		return
	}

	client := &Client{
		Conn:       conn,
		Identity:   reg.Identity,
		RemoteAddr: r.RemoteAddr,
	}

	// 如果已有同名客户端，先断开旧连接
	clientsMu.Lock()
	if old := clients[client.Identity]; old != nil {
		log.Printf("ℹ️ 发现同名客户端，关闭旧连接: %s (%s)", old.Identity, old.RemoteAddr)
		old.Conn.Close()
	}
	clients[client.Identity] = client
	clientsMu.Unlock()

	log.Printf("✅ 客户端注册: %s (来自 %s)", client.Identity, client.RemoteAddr)

	// 启动心跳协程
	go func() {
		ticker := time.NewTicker(pingPeriod)
		defer ticker.Stop()
		for {
			select {
			case <-ticker.C:
				client.Mu.Lock()
				_ = conn.SetWriteDeadline(time.Now().Add(writeWait))
				err := conn.WriteMessage(websocket.PingMessage, nil)
				client.Mu.Unlock()
				if err != nil {
					log.Printf("💔 心跳发送失败 (%s): %v", client.Identity, err)
					return
				}
			}
		}
	}()

	// 清理
	defer func() {
		clientsMu.Lock()
		if current := clients[client.Identity]; current == client {
			delete(clients, client.Identity)
		}
		clientsMu.Unlock()
		conn.Close() // 确保关闭
		log.Printf("🔌 客户端断开: %s", client.Identity)
	}()

	// 读取并转发消息
	for {
		var msg Message
		// 每次收到消息，也重置读超时 (双保险)
		_ = conn.SetReadDeadline(time.Now().Add(pongWait))

		if err := conn.ReadJSON(&msg); err != nil {
			if websocket.IsUnexpectedCloseError(err, websocket.CloseGoingAway, websocket.CloseAbnormalClosure) {
				log.Printf("❌ WebSocket 读取错误 (%s): %v", client.Identity, err)
			}
			break
		}

		clientsMu.RLock()
		target := clients[msg.To]
		clientsMu.RUnlock()

		if target == nil {
			// log.Printf("⚠️  目标不在线: %s", msg.To)
			continue
		}

		if err := safeWriteJSON(target, msg); err != nil {
			log.Printf("❌ 转发失败 (%s -> %s): %v", msg.From, msg.To, err)
			// 写失败通常表示连接异常，不用手动 close，心跳或下一次写会处理
		} else {
			log.Printf("✅ 消息已转发: %s -> %s (类型: %s)", msg.From, msg.To, msg.Type)
		}
	}
}

func handleStatus(w http.ResponseWriter, r *http.Request) {
	clientsMu.RLock()
	defer clientsMu.RUnlock()

	status := struct {
		OnlineClients []string `json:"online_clients"`
		Count         int      `json:"count"`
	}{
		OnlineClients: make([]string, 0, len(clients)),
		Count:         len(clients),
	}

	for id := range clients {
		status.OnlineClients = append(status.OnlineClients, id)
	}

	w.Header().Set("Content-Type", "application/json")
	_ = json.NewEncoder(w).Encode(status)
}

// safeWriteJSON 为单个连接加锁写入，避免 concurrent write panic，并设置写超时
func safeWriteJSON(c *Client, v interface{}) error {
	c.Mu.Lock()
	defer c.Mu.Unlock()

	_ = c.Conn.SetWriteDeadline(time.Now().Add(writeWait))
	return c.Conn.WriteJSON(v)
}
