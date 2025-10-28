package ros2

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"time"
)

// Client ROS2 客户端接口
type Client interface {
	// SubscribeImage 订阅图像话题
	SubscribeImage(topic string, callback func(*ROS2Image)) error
	
	// PublishControl 发布控制消息
	PublishControl(topic string, msg *UnifiedControlMessage) error
	
	// Close 关闭客户端
	Close() error
}

// SimpleClient 简化的 ROS2 客户端（使用命令行方式）
type SimpleClient struct {
	ctx            context.Context
	cancel         context.CancelFunc
	controlPub     *Publisher
	imageSub       *Subscriber
	mu             sync.Mutex
	imageCallbacks []func(*ROS2Image)
}

// NewSimpleClient 创建简化的 ROS2 客户端
func NewSimpleClient() (*SimpleClient, error) {
	ctx, cancel := context.WithCancel(context.Background())
	
	client := &SimpleClient{
		ctx:            ctx,
		cancel:         cancel,
		imageCallbacks: make([]func(*ROS2Image), 0),
	}
	
	return client, nil
}

// SubscribeImage 订阅图像话题
func (c *SimpleClient) SubscribeImage(topic string, callback func(*ROS2Image)) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	
	if c.imageSub != nil {
		return fmt.Errorf("图像订阅者已存在")
	}
	
	c.imageCallbacks = append(c.imageCallbacks, callback)
	
	sub, err := NewSubscriber(c.ctx, topic, "sensor_msgs/msg/Image")
	if err != nil {
		return fmt.Errorf("创建图像订阅者失败: %w", err)
	}
	
	c.imageSub = sub
	
	// 启动订阅处理
	go c.handleImageMessages()
	
	log.Printf("✅ 已订阅图像话题: %s", topic)
	return nil
}

// handleImageMessages 处理图像消息
func (c *SimpleClient) handleImageMessages() {
	for {
		select {
		case <-c.ctx.Done():
			return
		case data := <-c.imageSub.DataChan():
			// 解析 ROS2 图像消息
			var img ROS2Image
			if err := json.Unmarshal(data, &img); err != nil {
				log.Printf("⚠️ 解析图像消息失败: %v", err)
				continue
			}
			
			// 调用回调函数
			c.mu.Lock()
			callbacks := c.imageCallbacks
			c.mu.Unlock()
			
			for _, cb := range callbacks {
				go cb(&img)
			}
		}
	}
}

// PublishControl 发布控制消息
func (c *SimpleClient) PublishControl(topic string, msg *UnifiedControlMessage) error {
	c.mu.Lock()
	defer c.mu.Unlock()
	
	// 延迟创建发布者
	if c.controlPub == nil {
		pub, err := NewPublisher(c.ctx, topic, "std_msgs/msg/String")
		if err != nil {
			return fmt.Errorf("创建控制发布者失败: %w", err)
		}
		c.controlPub = pub
		log.Printf("✅ 已创建控制发布者: %s", topic)
	}
	
	// 序列化消息
	data, err := json.Marshal(msg)
	if err != nil {
		return fmt.Errorf("序列化控制消息失败: %w", err)
	}
	
	// 发布消息
	return c.controlPub.Publish(data)
}

// Close 关闭客户端
func (c *SimpleClient) Close() error {
	c.cancel()
	
	c.mu.Lock()
	defer c.mu.Unlock()
	
	if c.imageSub != nil {
		c.imageSub.Close()
	}
	
	if c.controlPub != nil {
		c.controlPub.Close()
	}
	
	log.Println("✅ ROS2 客户端已关闭")
	return nil
}

// TestROS2Available 测试 ROS2 是否可用
func TestROS2Available() bool {
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	
	// 尝试列出话题
	sub, err := NewSubscriber(ctx, "/test_topic_check", "std_msgs/msg/String")
	if err != nil {
		return false
	}
	sub.Close()
	
	return true
}

