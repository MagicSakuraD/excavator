#!/bin/bash

# 信令服务器启动脚本

set -e

cd "$(dirname "$0")/../bin"

# 检查端口是否被占用
PORT=8090
if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null 2>&1; then
    echo "⚠️  端口 $PORT 已被占用！正在停止旧进程..."
    pkill -f "signaling.*$PORT" || true
    sleep 1
fi

echo "🚀 启动信令服务器..."
echo "📡 监听端口: $PORT"
echo "📊 状态查询: http://$(hostname -I | awk '{print $1}'):$PORT/status"
echo "💡 按 Ctrl+C 停止"
echo ""

./signaling -addr :$PORT

