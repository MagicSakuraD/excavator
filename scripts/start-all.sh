#!/bin/bash

# 一键启动所有服务（后台运行）

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "🚀 启动所有服务..."
echo ""

# 停止旧进程
echo "1️⃣ 清理旧进程..."
pkill -f "signaling -addr" 2>/dev/null && echo "   ✅ 已停止旧的 signaling" || true
pkill -f "excavator -signaling" 2>/dev/null && echo "   ✅ 已停止旧的 excavator" || true
pkill -f "http.server 8080" 2>/dev/null && echo "   ✅ 已停止旧的 Web 服务器" || true
sleep 1

# 启动信令服务器
echo ""
echo "2️⃣ 启动信令服务器..."
cd bin
nohup ./signaling -addr :8090 > /tmp/signaling.log 2>&1 &
SIGNALING_PID=$!
sleep 2

if ! ps -p $SIGNALING_PID > /dev/null; then
    echo "❌ 信令服务器启动失败，查看日志: /tmp/signaling.log"
    exit 1
fi
echo "   ✅ 信令服务器启动成功 (PID: $SIGNALING_PID)"
echo "   📝 日志: /tmp/signaling.log"

# 启动挖掘机程序
echo ""
echo "3️⃣ 启动挖掘机程序..."
nohup ./excavator \
    -signaling ws://$LOCAL_IP:8090/ws \
    -camera /dev/video0 \
    -width 640 \
    -height 480 \
    -fps 30 \
    > /tmp/excavator.log 2>&1 &
EXCAVATOR_PID=$!
sleep 2

if ! ps -p $EXCAVATOR_PID > /dev/null; then
    echo "❌ 挖掘机程序启动失败，查看日志: /tmp/excavator.log"
    kill $SIGNALING_PID 2>/dev/null || true
    exit 1
fi
echo "   ✅ 挖掘机程序启动成功 (PID: $EXCAVATOR_PID)"
echo "   📝 日志: /tmp/excavator.log"

# 启动 Web 服务器
echo ""
echo "4️⃣ 启动 Web 服务器..."
cd "$PROJECT_ROOT/web"
nohup python3 -m http.server 8080 > /tmp/webserver.log 2>&1 &
WEB_PID=$!
sleep 1

if ! ps -p $WEB_PID > /dev/null; then
    echo "❌ Web 服务器启动失败，查看日志: /tmp/webserver.log"
    kill $SIGNALING_PID $EXCAVATOR_PID 2>/dev/null || true
    exit 1
fi
echo "   ✅ Web 服务器启动成功 (PID: $WEB_PID)"
echo "   📝 日志: /tmp/webserver.log"

# 显示访问信息
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ 所有服务启动成功！"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📡 本机访问:"
echo "   http://localhost:8080/controller.html"
echo ""
echo "🌍 局域网访问 (其他电脑):"
echo "   http://$LOCAL_IP:8080/controller.html"
echo ""
echo "🛑 停止所有服务:"
echo "   ./scripts/kill-all.sh"
echo ""
echo "📝 查看日志:"
echo "   信令服务器: tail -f /tmp/signaling.log"
echo "   挖掘机程序: tail -f /tmp/excavator.log"
echo "   Web 服务器: tail -f /tmp/webserver.log"
echo ""
echo "💡 提示: 服务已在后台运行，可以关闭此终端"
echo ""

# 保存 PID 到文件
echo "$SIGNALING_PID" > /tmp/excavator_signaling.pid
echo "$EXCAVATOR_PID" > /tmp/excavator_excavator.pid
echo "$WEB_PID" > /tmp/excavator_web.pid

