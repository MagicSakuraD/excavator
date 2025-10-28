#!/bin/bash

# 启动 Web 服务器 - 用于局域网访问控制页面

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT/web"

# 获取本机 IP
LOCAL_IP=$(hostname -I | awk '{print $1}')
PORT=8080

# 检查端口是否被占用
if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo "⚠️  端口 $PORT 已被占用！"
    read -p "是否停止占用该端口的进程？(y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        lsof -ti:$PORT | xargs kill -9 2>/dev/null || true
        sleep 1
    else
        echo "❌ 请先停止占用端口的进程"
        exit 1
    fi
fi

echo "🌐 启动 Web 服务器..."
echo ""
echo "📡 本地访问:"
echo "   http://localhost:$PORT/controller.html"
echo ""
echo "🌍 局域网访问 (其他电脑):"
echo "   http://$LOCAL_IP:$PORT/controller.html"
echo ""
echo "⚠️  重要提示:"
echo "   1. 确保已修改 controller.html 中的信令服务器地址"
echo "   2. 将 'ws://localhost:8090/ws' 改为 'ws://$LOCAL_IP:8090/ws'"
echo "   3. 确保防火墙已开放 8080 和 8090 端口"
echo ""
echo "💡 按 Ctrl+C 停止服务器"
echo ""

python3 -m http.server $PORT

