package ros2

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"log"
	"os/exec"
	"strings"
	"time"
)

// Publisher ROS2 发布者
type Publisher struct {
	ctx       context.Context
	topic     string
	msgType   string
	cmdCancel context.CancelFunc
}

// NewPublisher 创建发布者
func NewPublisher(ctx context.Context, topic, msgType string) (*Publisher, error) {
	pub := &Publisher{
		ctx:     ctx,
		topic:   topic,
		msgType: msgType,
	}
	
	return pub, nil
}

// Publish 发布消息（使用 ros2 topic pub）
func (p *Publisher) Publish(data []byte) error {
	// 将 JSON 数据包装为 ROS2 String 消息格式
	var jsonMsg map[string]interface{}
	if err := json.Unmarshal(data, &jsonMsg); err != nil {
		return fmt.Errorf("解析 JSON 失败: %w", err)
	}
	
	// 转换为 ROS2 CLI 格式的 YAML
	dataStr := string(data)
	msgYAML := fmt.Sprintf("{data: '%s'}", strings.ReplaceAll(dataStr, "'", "''"))
	
	// 使用 ros2 topic pub --once 发布一次
	ctx, cancel := context.WithTimeout(p.ctx, 1*time.Second)
	defer cancel()
	
	cmd := exec.CommandContext(ctx, "ros2", "topic", "pub", "--once", p.topic, p.msgType, msgYAML)
	
	var stderr bytes.Buffer
	cmd.Stderr = &stderr
	
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("发布消息失败: %w, stderr: %s", err, stderr.String())
	}
	
	return nil
}

// Close 关闭发布者
func (p *Publisher) Close() error {
	if p.cmdCancel != nil {
		p.cmdCancel()
	}
	return nil
}

// Subscriber ROS2 订阅者
type Subscriber struct {
	ctx       context.Context
	cancel    context.CancelFunc
	topic     string
	msgType   string
	dataChan  chan []byte
	cmd       *exec.Cmd
}

// NewSubscriber 创建订阅者
func NewSubscriber(ctx context.Context, topic, msgType string) (*Subscriber, error) {
	subCtx, cancel := context.WithCancel(ctx)
	
	sub := &Subscriber{
		ctx:      subCtx,
		cancel:   cancel,
		topic:    topic,
		msgType:  msgType,
		dataChan: make(chan []byte, 100),
	}
	
	// 启动订阅进程
	go sub.startSubscription()
	
	return sub, nil
}

// startSubscription 启动订阅（使用 ros2 topic echo）
func (s *Subscriber) startSubscription() {
	defer close(s.dataChan)
	
	for {
		select {
		case <-s.ctx.Done():
			return
		default:
			// 创建 ros2 topic echo 命令
			cmd := exec.CommandContext(s.ctx, "ros2", "topic", "echo", "--once", s.topic, s.msgType)
			
			var stdout, stderr bytes.Buffer
			cmd.Stdout = &stdout
			cmd.Stderr = &stderr
			
			if err := cmd.Run(); err != nil {
				if s.ctx.Err() != nil {
					// 上下文已取消，正常退出
					return
				}
				log.Printf("⚠️ 订阅话题 %s 失败: %v, stderr: %s", s.topic, err, stderr.String())
				time.Sleep(1 * time.Second) // 避免快速重试
				continue
			}
			
			// 解析输出
			data := stdout.Bytes()
			if len(data) > 0 {
				// 将 YAML 转换为 JSON（简化处理）
				jsonData, err := yamlToJSON(data)
				if err != nil {
					log.Printf("⚠️ 转换消息格式失败: %v", err)
					continue
				}
				
				select {
				case s.dataChan <- jsonData:
				case <-s.ctx.Done():
					return
				}
			}
			
			// 短暂延迟避免 CPU 占用过高
			time.Sleep(10 * time.Millisecond)
		}
	}
}

// DataChan 获取数据通道
func (s *Subscriber) DataChan() <-chan []byte {
	return s.dataChan
}

// Close 关闭订阅者
func (s *Subscriber) Close() error {
	s.cancel()
	
	// 等待通道关闭
	for range s.dataChan {
		// 清空通道
	}
	
	return nil
}

// yamlToJSON 简化的 YAML 到 JSON 转换（仅用于 ROS2 消息）
func yamlToJSON(yamlData []byte) ([]byte, error) {
	// 这是一个简化实现
	// 实际应该使用 yaml 库进行完整解析
	
	// 对于 std_msgs/String，格式是: data: "..."
	yamlStr := string(yamlData)
	
	// 简单提取 data 字段
	if strings.Contains(yamlStr, "data:") {
		parts := strings.SplitN(yamlStr, "data:", 2)
		if len(parts) == 2 {
			dataValue := strings.TrimSpace(parts[1])
			dataValue = strings.Trim(dataValue, "'\"")
			
			// 尝试解析为 JSON
			var jsonObj interface{}
			if err := json.Unmarshal([]byte(dataValue), &jsonObj); err == nil {
				return []byte(dataValue), nil
			}
			
			// 如果不是 JSON，包装为简单对象
			result := map[string]string{"data": dataValue}
			return json.Marshal(result)
		}
	}
	
	// 对于 sensor_msgs/Image，尝试直接解析
	// 这需要更复杂的 YAML 解析，暂时返回错误
	return nil, fmt.Errorf("不支持的消息格式")
}

