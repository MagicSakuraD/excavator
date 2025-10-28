package ros2

import "time"

// UnifiedControlMessage 统一控制消息结构（与 Python/Rust 版本保持一致）
type UnifiedControlMessage struct {
	// 装载机专用控制
	Rotation float64 `json:"rotation"` // 方向盘旋转: -1 (左) to 1 (右)
	Brake    float64 `json:"brake"`    // 刹车: 0 (松开) to 1 (踩死)
	Throttle float64 `json:"throttle"` // 油门: 0 (松开) to 1 (踩死)
	Gear     string  `json:"gear"`     // 档位: 'P' | 'R' | 'N' | 'D'

	// 共用控制
	Boom   float64 `json:"boom"`   // 大臂: -1 (降) to 1 (提)
	Bucket float64 `json:"bucket"` // 铲斗: -1 (收) to 1 (翻)

	// 兼容性属性
	LeftTrack  float64 `json:"left_track"`  // 左履带: -1 (后) to 1 (前)
	RightTrack float64 `json:"right_track"` // 右履带: -1 (后) to 1 (前)
	Swing      float64 `json:"swing"`       // 驾驶室旋转: -1 (左) to 1 (右)
	Stick      float64 `json:"stick"`       // 小臂: -1 (收) to 1 (伸)

	// 设备类型标识
	DeviceType string `json:"device_type"`
	Timestamp  int64  `json:"timestamp"`
}

// NewDefaultControlMessage 创建默认控制消息
func NewDefaultControlMessage() *UnifiedControlMessage {
	return &UnifiedControlMessage{
		Rotation:   0.0,
		Brake:      0.0,
		Throttle:   0.0,
		Gear:       "N",
		Boom:       0.0,
		Bucket:     0.0,
		LeftTrack:  0.0,
		RightTrack: 0.0,
		Swing:      0.0,
		Stick:      0.0,
		DeviceType: "wheel_loader",
		Timestamp:  time.Now().UnixMilli(),
	}
}

// ControlMessage WebRTC 控制消息（从浏览器接收）
type ControlMessage struct {
	Type string                 `json:"type"` // "gear" | "analog"
	Gear string                 `json:"gear,omitempty"`
	V    map[string]float64     `json:"v,omitempty"`
	T    int64                  `json:"t"`
}

// ToUnifiedControl 转换为统一控制消息
func (c *ControlMessage) ToUnifiedControl() *UnifiedControlMessage {
	msg := NewDefaultControlMessage()
	msg.Timestamp = c.T

	switch c.Type {
	case "gear":
		msg.Gear = c.Gear
	case "analog":
		if c.V != nil {
			if v, ok := c.V["rotation"]; ok {
				msg.Rotation = v
			}
			if v, ok := c.V["brake"]; ok {
				msg.Brake = v
			}
			if v, ok := c.V["throttle"]; ok {
				msg.Throttle = v
			}
			if v, ok := c.V["boom"]; ok {
				msg.Boom = v
			}
			if v, ok := c.V["bucket"]; ok {
				msg.Bucket = v
			}
			if v, ok := c.V["leftTrack"]; ok {
				msg.LeftTrack = v
			}
			if v, ok := c.V["rightTrack"]; ok {
				msg.RightTrack = v
			}
			if v, ok := c.V["swing"]; ok {
				msg.Swing = v
			}
			if v, ok := c.V["stick"]; ok {
				msg.Stick = v
			}
		}
	}

	return msg
}

// ROS2ImageHeader ROS2 图像消息头
type ROS2ImageHeader struct {
	Stamp struct {
		Sec  int32 `json:"sec"`
		Nsec int32 `json:"nsec"`
	} `json:"stamp"`
	FrameID string `json:"frame_id"`
}

// ROS2Image ROS2 图像消息（简化版 sensor_msgs/Image）
type ROS2Image struct {
	Header   ROS2ImageHeader `json:"header"`
	Height   uint32          `json:"height"`
	Width    uint32          `json:"width"`
	Encoding string          `json:"encoding"` // "rgb8", "bgr8", "mono8", etc.
	IsBigEndian uint8       `json:"is_bigendian"`
	Step     uint32          `json:"step"` // 每行字节数
	Data     []byte          `json:"data"`
}

