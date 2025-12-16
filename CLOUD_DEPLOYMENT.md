# ☁️ 云服务器部署指南

本文档说明如何将信令服务器部署到云服务器（如阿里云、腾讯云、AWS 等）。

---

## 📋 部署架构

### 原架构（本地测试）
```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│  浏览器      │────►│ 信令服务器    │◄────│  Orin 设备   │
│ 控制端       │     │ (localhost)  │     │ 挖掘机程序   │
└─────────────┘     └──────────────┘     └─────────────┘
    同一局域网（192.168.3.x）
```

### 云部署架构（推荐）
```
                    ┌──────────────────┐
                    │   云服务器        │
                    │  (公网 IP)       │
                    │  信令服务器       │
                    │  8090 端口        │
                    └────────┬─────────┘
                             │
            ┌────────────────┼────────────────┐
            │                │                │
            ▼                ▼                ▼
    ┌─────────────┐  ┌──────────────┐  ┌─────────────┐
    │  控制端 A    │  │  控制端 B     │  │  Orin 设备   │
    │ (任何位置)   │  │  (任何位置)   │  │  (工地现场)  │
    └─────────────┘  └──────────────┘  └─────────────┘
        家里/办公室      移动端            4G/5G/WiFi
```

**优势**：
- ✅ 控制端可以从任何地方访问
- ✅ 多个控制端可以同时监控
- ✅ 不需要公网 IP 或端口映射
- ✅ 信令服务器作为中继，更稳定

---

## 🖥️ 云服务器要求

### 最低配置
- **CPU**: 1 核
- **内存**: 1GB
- **带宽**: 1Mbps（信令服务器本身流量很小）
- **系统**: Ubuntu 20.04/22.04 或 CentOS 7/8

### 推荐配置
- **CPU**: 2 核
- **内存**: 2GB
- **带宽**: 3Mbps（支持更多并发连接）
- **系统**: Ubuntu 22.04 LTS

### 云服务商选择
- 阿里云 ECS
- 腾讯云 CVM
- AWS EC2
- 华为云 ECS
- DigitalOcean Droplet

**注意**：选择离 Orin 设备（工地）较近的区域，可降低延迟。

---

## 🚀 部署步骤

### 第 1 步：购买和配置云服务器

1. **购买云服务器**
   - 选择合适的区域（如：华东、华南）
   - 选择 Ubuntu 22.04 系统
   - 配置安全组/防火墙（见下方）

2. **记录公网 IP**
   ```
   假设你的云服务器公网 IP 是: 47.98.123.45
   ```

3. **SSH 登录云服务器**
   ```bash
   ssh root@47.98.123.45
   # 或使用密钥
   ssh -i your-key.pem ubuntu@47.98.123.45
   ```

---

### 第 2 步：配置防火墙/安全组

**重要**：必须开放以下端口

| 端口 | 协议 | 说明 | 必需 |
|-----|------|------|------|
| 22 | TCP | SSH 远程管理 | ✅ |
| 8090 | TCP | WebSocket 信令服务器 | ✅ |
| 8080 | TCP | Web 控制页面（可选） | ⭕ |

#### 阿里云/腾讯云配置

**在云控制台：**
1. 进入 **安全组** 设置
2. 添加入站规则：
   - 端口：8090
   - 协议：TCP
   - 源地址：0.0.0.0/0（允许所有）
   - 描述：WebRTC 信令服务器

**在服务器内部（Ubuntu）：**
```bash
# 检查防火墙状态
sudo ufw status

# 如果防火墙启用，开放端口
sudo ufw allow 22/tcp
sudo ufw allow 8090/tcp
sudo ufw allow 8080/tcp  # 如果需要 Web 页面

# 重启防火墙
sudo ufw reload
```

---

### 第 3 步：安装和部署信令服务器

#### 方式 1：从本地编译并上传（推荐）

**在本地 Orin 设备上：**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator

# 为 Linux x86_64 交叉编译（如果云服务器是 x86_64）
GOOS=linux GOARCH=amd64 go build -o bin/signaling-amd64 ./cmd/signaling

# 如果云服务器也是 ARM64（如华为鲲鹏）
GOOS=linux GOARCH=arm64 go build -o bin/signaling-arm64 ./cmd/signaling

# 上传到云服务器
scp bin/signaling-amd64 root@47.98.123.45:/root/signaling
```

#### 方式 2：在云服务器上编译

**在云服务器上：**

```bash
# 安装 Go（如果没有）
wget https://go.dev/dl/go1.21.5.linux-amd64.tar.gz
sudo tar -C /usr/local -xzf go1.21.5.linux-amd64.tar.gz
export PATH=$PATH:/usr/local/go/bin
echo 'export PATH=$PATH:/usr/local/go/bin' >> ~/.bashrc

# 克隆代码（或上传代码）
mkdir -p ~/excavator
cd ~/excavator

# 假设你已经上传了代码
# 或者使用 git clone（如果代码在 GitHub）

# 编译
export GOPROXY=https://goproxy.cn,direct
go build -o signaling ./cmd/signaling/main.go
```

---

### 第 4 步：创建启动脚本

**在云服务器上创建 `/root/start-signaling.sh`：**

```bash
cat > /root/start-signaling.sh << 'EOF'
#!/bin/bash

# 信令服务器启动脚本

set -e

SIGNALING_BIN="/root/signaling"
LISTEN_ADDR=":8090"
LOG_FILE="/var/log/signaling.log"

# 检查可执行文件
if [ ! -f "$SIGNALING_BIN" ]; then
    echo "❌ 找不到信令服务器: $SIGNALING_BIN"
    exit 1
fi

# 创建日志目录
sudo mkdir -p $(dirname $LOG_FILE)

echo "🚀 启动信令服务器..."
echo "📡 监听地址: $LISTEN_ADDR"
echo "📝 日志文件: $LOG_FILE"

# 启动服务器
nohup $SIGNALING_BIN -addr $LISTEN_ADDR >> $LOG_FILE 2>&1 &

PID=$!
echo "✅ 信令服务器已启动 (PID: $PID)"
echo "$PID" > /var/run/signaling.pid

# 等待服务器启动
sleep 2

# 检查进程
if ps -p $PID > /dev/null; then
    echo "✅ 服务运行正常"
    echo ""
    echo "📊 状态查询:"
    echo "   curl http://localhost:8090/status"
    echo ""
    echo "🛑 停止服务:"
    echo "   kill $PID"
else
    echo "❌ 服务启动失败，查看日志:"
    echo "   tail -f $LOG_FILE"
    exit 1
fi
EOF

# 添加执行权限
chmod +x /root/start-signaling.sh
```

---

### 第 5 步：配置系统服务（可选，推荐）

使用 systemd 实现开机自启动和自动重启。

**创建 `/etc/systemd/system/signaling.service`：**

```bash
sudo tee /etc/systemd/system/signaling.service > /dev/null << 'EOF'
[Unit]
Description=WebRTC Signaling Server
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/root
ExecStart=/root/signaling -addr :8090
Restart=always
RestartSec=5
StandardOutput=append:/var/log/signaling.log
StandardError=append:/var/log/signaling.log

[Install]
WantedBy=multi-user.target
EOF

# 重载 systemd
sudo systemctl daemon-reload

# 启动服务
sudo systemctl start signaling

# 查看状态
sudo systemctl status signaling

# 设置开机自启动
sudo systemctl enable signaling
```

**systemd 服务管理命令：**

```bash
# 启动
sudo systemctl start signaling

# 停止
sudo systemctl stop signaling

# 重启
sudo systemctl restart signaling

# 查看状态
sudo systemctl status signaling

# 查看日志
sudo journalctl -u signaling -f

# 或
tail -f /var/log/signaling.log
```

---

### 第 6 步：验证部署

**在云服务器上：**

```bash
# 检查端口监听
sudo netstat -tlnp | grep 8090
# 或
sudo ss -tlnp | grep 8090

# 应该看到类似：
# tcp6       0      0 :::8090                 :::*                    LISTEN      12345/signaling

# 测试本地连接
curl http://localhost:8090/status
# 应该返回 JSON 响应

# 检查进程
ps aux | grep signaling
```

**从本地测试连接：**

```bash
# 替换为你的云服务器公网 IP
curl http://47.98.123.45:8090/status

# 或使用 WebSocket 客户端测试
wscat -c ws://47.98.123.45:8090/ws
```

---

## 🔧 客户端配置更新

### 1. 更新 Orin 设备（挖掘机端）

**修改启动脚本或直接运行：**

```bash
# 之前（本地）
./bin/excavator -signaling ws://127.0.0.1:8090/ws

# 现在（云服务器）
./bin/excavator -signaling ws://47.98.123.45:8090/ws \
  -camera /dev/video0 \
  -enable-ros2 true
```

**或更新 `scripts/start-excavator.sh`：**

```bash
# 修改这一行
SIGNALING_SERVER="ws://47.98.123.45:8090/ws"
```

### 2. 更新控制端网页

**方式 1：修改 `controller.html`**

```bash
cd /home/orin64/MyCode/PionWebrtc/excavator/web
nano controller.html

# 找到这一行（约 193 行）：
const SIGNALING_SERVER = 'ws://192.168.0.87:8090/ws';

# 改为：
const SIGNALING_SERVER = 'ws://47.98.123.45:8090/ws';
```

**方式 2：将网页也部署到云服务器**

```bash
# 在云服务器上
mkdir -p /var/www/excavator
cd /var/www/excavator

# 从本地上传 controller.html
scp ~/MyCode/PionWebrtc/excavator/web/controller.html root@47.98.123.45:/var/www/excavator/

# 启动简单的 HTTP 服务器
cd /var/www/excavator
python3 -m http.server 8080 &

# 或使用 nginx（推荐生产环境）
sudo apt install nginx
sudo cp controller.html /var/www/html/
```

然后访问：`http://47.98.123.45:8080/controller.html`

---

## 🔐 安全性建议

### 1. 启用 HTTPS/WSS（强烈推荐）

使用 Let's Encrypt 免费证书：

```bash
# 安装 certbot
sudo apt install certbot

# 获取证书（需要域名）
sudo certbot certonly --standalone -d your-domain.com

# 证书位置
# /etc/letsencrypt/live/your-domain.com/fullchain.pem
# /etc/letsencrypt/live/your-domain.com/privkey.pem
```

**修改信令服务器支持 TLS（需要修改代码）**：

```go
// cmd/signaling/main.go
// 添加 TLS 支持
http.ListenAndServeTLS(":8090", "cert.pem", "key.pem", nil)
```

### 2. 添加认证（推荐）

**简单的 Token 认证：**

修改 `controller.html`：
```javascript
const SIGNALING_SERVER = 'ws://47.98.123.45:8090/ws?token=your-secret-token';
```

修改 `cmd/signaling/main.go` 添加 token 验证逻辑。

### 3. 限制来源 IP（可选）

在云服务器安全组只允许特定 IP：

```bash
# 阿里云/腾讯云控制台
# 安全组规则 > 添加
# 端口：8090
# 源地址：123.45.67.89/32  （你的固定 IP）
```

### 4. 定期更新

```bash
# 定期更新系统
sudo apt update && sudo apt upgrade -y

# 重新编译和部署最新代码
```

---

## 📊 监控和运维

### 查看日志

```bash
# 实时查看日志
tail -f /var/log/signaling.log

# 或使用 journalctl
sudo journalctl -u signaling -f

# 查看最近 100 行
sudo journalctl -u signaling -n 100
```

### 性能监控

```bash
# CPU 和内存占用
top -p $(cat /var/run/signaling.pid)

# 网络连接数
netstat -an | grep 8090 | wc -l

# 带宽使用
sudo iftop -i eth0
```

### 设置告警（可选）

使用云服务商的监控服务：
- 阿里云：云监控
- 腾讯云：云监控
- AWS：CloudWatch

监控指标：
- CPU 使用率 > 80%
- 内存使用率 > 80%
- 进程存活状态

---

## 🐛 故障排查

### 问题 1：无法连接到信令服务器

**检查项：**

1. **云服务器防火墙**
   ```bash
   sudo ufw status
   sudo ufw allow 8090/tcp
   ```

2. **云服务商安全组**
   - 检查是否开放 8090 端口
   - 源地址是否设置为 0.0.0.0/0

3. **服务是否运行**
   ```bash
   sudo systemctl status signaling
   ps aux | grep signaling
   ```

4. **测试本地连接**
   ```bash
   # 在云服务器上
   curl http://localhost:8090/status
   ```

5. **测试外网连接**
   ```bash
   # 在本地
   curl http://47.98.123.45:8090/status
   ```

### 问题 2：服务启动失败

**查看日志：**
```bash
tail -f /var/log/signaling.log
sudo journalctl -u signaling -n 50
```

**常见原因：**
- 端口被占用：`sudo lsof -i:8090`
- 权限问题：检查可执行文件权限
- 依赖库缺失：重新编译

### 问题 3：WebSocket 连接断开

**可能原因：**
- 云服务器 NAT 超时（调整超时设置）
- 网络不稳定（使用 STUN/TURN）
- 客户端网络切换（实现重连机制）

---

## 📈 性能优化

### 增加并发连接数

```bash
# 修改系统限制
sudo vim /etc/security/limits.conf

# 添加：
* soft nofile 65535
* hard nofile 65535

# 重启系统或重新登录
```

### 使用 Nginx 反向代理

```nginx
# /etc/nginx/sites-available/signaling
upstream signaling {
    server 127.0.0.1:8090;
}

server {
    listen 80;
    server_name your-domain.com;

    location /ws {
        proxy_pass http://signaling;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
    }

    location /status {
        proxy_pass http://signaling;
    }
}
```

---

## 📝 完整部署检查清单

- [ ] 购买云服务器并记录公网 IP
- [ ] 配置安全组开放 8090 端口
- [ ] SSH 登录云服务器
- [ ] 上传或编译信令服务器程序
- [ ] 创建启动脚本
- [ ] 配置 systemd 服务
- [ ] 启动服务并验证
- [ ] 测试外网连接
- [ ] 更新 Orin 设备配置
- [ ] 更新控制端网页配置
- [ ] 测试完整流程
- [ ] 配置日志和监控
- [ ] 设置开机自启动

---

## 🎯 快速部署脚本

**一键部署脚本（在云服务器上运行）：**

```bash
#!/bin/bash
# 快速部署信令服务器

set -e

echo "🚀 开始部署信令服务器..."

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <信令服务器可执行文件路径>"
    exit 1
fi

SIGNALING_BIN="$1"

# 检查文件
if [ ! -f "$SIGNALING_BIN" ]; then
    echo "❌ 文件不存在: $SIGNALING_BIN"
    exit 1
fi

# 复制文件
sudo cp "$SIGNALING_BIN" /usr/local/bin/signaling
sudo chmod +x /usr/local/bin/signaling

# 创建 systemd 服务
sudo tee /etc/systemd/system/signaling.service > /dev/null << 'EOF'
[Unit]
Description=WebRTC Signaling Server
After=network.target

[Service]
Type=simple
ExecStart=/usr/local/bin/signaling -addr :8090
Restart=always
RestartSec=5
StandardOutput=append:/var/log/signaling.log
StandardError=append:/var/log/signaling.log

[Install]
WantedBy=multi-user.target
EOF

# 启动服务
sudo systemctl daemon-reload
sudo systemctl enable signaling
sudo systemctl start signaling

# 等待启动
sleep 2

# 检查状态
sudo systemctl status signaling

echo ""
echo "✅ 部署完成！"
echo ""
echo "📊 查看状态: sudo systemctl status signaling"
echo "📝 查看日志: tail -f /var/log/signaling.log"
echo "🌐 访问地址: ws://$(curl -s ifconfig.me):8090/ws"
```

保存为 `deploy.sh` 并运行：
```bash
chmod +x deploy.sh
./deploy.sh /path/to/signaling
```

---

## 🔗 相关文档

- [README.md](./README.md) - 项目概述
- [EXAMPLES.md](./EXAMPLES.md) - 使用示例
- [ROS2_INTEGRATION.md](./ROS2_INTEGRATION.md) - ROS2 集成

---

**部署完成后，记得更新所有客户端的信令服务器地址！** 🎉

