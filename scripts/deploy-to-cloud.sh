#!/bin/bash

# 云服务器部署脚本
# 在本地 Orin 设备上运行，将信令服务器部署到云服务器

set -e

echo "☁️  云服务器部署工具"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# 检查参数
if [ $# -lt 1 ]; then
    echo "用法: $0 <云服务器地址> [SSH用户名] [SSH端口]"
    echo ""
    echo "示例:"
    echo "  $0 47.98.123.45"
    echo "  $0 47.98.123.45 root 22"
    echo "  $0 user@47.98.123.45"
    echo ""
    exit 1
fi

CLOUD_HOST="$1"
SSH_USER="${2:-root}"
SSH_PORT="${3:-22}"

# 如果地址包含 @，提取用户名
if [[ "$CLOUD_HOST" == *"@"* ]]; then
    SSH_USER="${CLOUD_HOST%%@*}"
    CLOUD_HOST="${CLOUD_HOST##*@}"
fi

echo "📋 部署配置:"
echo "   云服务器: $CLOUD_HOST"
echo "   SSH 用户: $SSH_USER"
echo "   SSH 端口: $SSH_PORT"
echo ""

# 确认
read -p "确认开始部署？(y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ 已取消"
    exit 1
fi

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

echo ""
echo "1️⃣ 检查本地编译文件..."

if [ ! -f "bin/signaling" ]; then
    echo "⚠️  未找到编译文件，开始编译..."
    export GOPROXY=https://goproxy.cn,direct
    
    # 检测云服务器架构（默认 amd64）
    echo "   编译目标: Linux AMD64"
    GOOS=linux GOARCH=amd64 go build -o bin/signaling-cloud ./cmd/signaling
    SIGNALING_BIN="bin/signaling-cloud"
else
    echo "   使用现有编译文件: bin/signaling"
    SIGNALING_BIN="bin/signaling"
fi

echo "✅ 编译文件准备完成"
echo ""

echo "2️⃣ 测试 SSH 连接..."
if ! ssh -p "$SSH_PORT" -o ConnectTimeout=5 "$SSH_USER@$CLOUD_HOST" "echo '连接成功'" 2>/dev/null; then
    echo "❌ SSH 连接失败"
    echo "请检查:"
    echo "  1. 云服务器地址是否正确"
    echo "  2. SSH 密钥或密码是否配置"
    echo "  3. 云服务器是否允许 SSH 连接"
    exit 1
fi
echo "✅ SSH 连接正常"
echo ""

echo "3️⃣ 上传文件到云服务器..."
scp -P "$SSH_PORT" "$SIGNALING_BIN" "$SSH_USER@$CLOUD_HOST:/tmp/signaling"
echo "✅ 文件上传完成"
echo ""

echo "4️⃣ 在云服务器上配置服务..."

# 创建远程部署脚本
REMOTE_SCRIPT=$(cat << 'REMOTE_EOF'
#!/bin/bash
set -e

echo "📦 安装信令服务器..."

# 移动文件
sudo mv /tmp/signaling /usr/local/bin/signaling
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

# 重载并启动服务
sudo systemctl daemon-reload
sudo systemctl enable signaling
sudo systemctl restart signaling

# 等待启动
sleep 2

# 检查状态
if sudo systemctl is-active --quiet signaling; then
    echo "✅ 服务启动成功"
else
    echo "❌ 服务启动失败"
    sudo systemctl status signaling
    exit 1
fi

# 创建日志目录
sudo mkdir -p /var/log

echo ""
echo "✅ 部署完成！"
REMOTE_EOF
)

# 执行远程脚本
ssh -p "$SSH_PORT" "$SSH_USER@$CLOUD_HOST" "bash -s" <<< "$REMOTE_SCRIPT"

echo ""
echo "5️⃣ 验证部署..."

# 测试服务
sleep 2
if ssh -p "$SSH_PORT" "$SSH_USER@$CLOUD_HOST" "curl -s http://localhost:8090/status" > /dev/null 2>&1; then
    echo "✅ 服务运行正常"
else
    echo "⚠️  本地连接测试失败，但服务可能仍在启动中"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎉 部署成功！"
echo ""
echo "📋 重要信息:"
echo "   云服务器 IP: $CLOUD_HOST"
echo "   信令地址: ws://$CLOUD_HOST:8090/ws"
echo "   状态查询: http://$CLOUD_HOST:8090/status"
echo ""
echo "⚠️  下一步操作:"
echo ""
echo "1️⃣ 配置云服务器防火墙/安全组:"
echo "   - 开放端口: 8090/TCP"
echo "   - 源地址: 0.0.0.0/0（允许所有）"
echo ""
echo "2️⃣ 更新客户端配置:"
echo ""
echo "   Orin 设备 (挖掘机端):"
echo "   ./bin/excavator -signaling ws://$CLOUD_HOST:8090/ws"
echo ""
echo "   或修改脚本:"
echo "   编辑 scripts/start-excavator.sh"
echo "   SIGNALING_SERVER=\"ws://$CLOUD_HOST:8090/ws\""
echo ""
echo "   控制端网页:"
echo "   编辑 web/controller.html"
echo "   const SIGNALING_SERVER = 'ws://$CLOUD_HOST:8090/ws';"
echo ""
echo "3️⃣ 测试连接:"
echo "   curl http://$CLOUD_HOST:8090/status"
echo ""
echo "4️⃣ 查看日志:"
echo "   ssh $SSH_USER@$CLOUD_HOST 'tail -f /var/log/signaling.log'"
echo ""
echo "5️⃣ 管理服务:"
echo "   启动: ssh $SSH_USER@$CLOUD_HOST 'sudo systemctl start signaling'"
echo "   停止: ssh $SSH_USER@$CLOUD_HOST 'sudo systemctl stop signaling'"
echo "   状态: ssh $SSH_USER@$CLOUD_HOST 'sudo systemctl status signaling'"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

