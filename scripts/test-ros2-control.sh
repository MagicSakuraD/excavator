#!/bin/bash

# 快速测试 ROS2 控制功能

set -e

cd "$(dirname "$0")/.."

echo "🧪 测试 ROS2 控制消息发布"
echo ""

# 检查 ROS2
if ! command -v ros2 >/dev/null 2>&1; then
    echo "❌ ROS2 未安装或未 source 环境"
    echo "请运行: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS2 环境就绪"
echo ""

# 停止旧进程
echo "🧹 清理旧进程..."
pkill -f "signaling -addr" 2>/dev/null || true
pkill -f "excavator -signaling" 2>/dev/null || true
sleep 1

# 启动信令服务器（后台）
echo "1️⃣ 启动信令服务器..."
./bin/signaling -addr :8090 > /tmp/signaling.log 2>&1 &
SIGNALING_PID=$!
sleep 2

# 启动挖掘机程序（后台，启用 ROS2）
echo "2️⃣ 启动挖掘机程序（启用 ROS2）..."
./bin/excavator \
    -signaling ws://127.0.0.1:8090/ws \
    -camera /dev/video0 \
    -enable-ros2 true \
    -ros2-control-topic /controls/teleop \
    > /tmp/excavator.log 2>&1 &
EXCAVATOR_PID=$!
sleep 2

# 在后台监听 ROS2 话题
echo "3️⃣ 监听 ROS2 控制话题..."
echo ""

# 创建临时监听脚本
MONITOR_SCRIPT="/tmp/ros2_monitor.sh"
cat > "$MONITOR_SCRIPT" << 'EOF'
#!/bin/bash
echo "📡 监听 /controls/teleop..."
echo "等待控制消息..."
echo ""
timeout 60 ros2 topic echo /controls/teleop std_msgs/msg/String 2>/dev/null || true
EOF
chmod +x "$MONITOR_SCRIPT"

# 在后台运行监听
"$MONITOR_SCRIPT" &
MONITOR_PID=$!

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ 服务已启动！"
echo ""
echo "📋 进程 ID:"
echo "   信令服务器: $SIGNALING_PID"
echo "   挖掘机程序: $EXCAVATOR_PID"
echo ""
echo "📝 日志文件:"
echo "   信令服务器: /tmp/signaling.log"
echo "   挖掘机程序: /tmp/excavator.log"
echo ""
echo "🌐 测试步骤:"
echo "   1. 打开浏览器: http://localhost:8080/controller.html"
echo "      (需要先启动 Web 服务器: cd web && python3 -m http.server 8080)"
echo ""
echo "   2. 点击 '连接挖掘机'"
echo ""
echo "   3. 打开浏览器 Console (F12)，发送测试指令:"
echo ""
echo "      dataChannel.send(JSON.stringify({"
echo "        type: 'analog',"
echo "        v: { rotation: 0.5, throttle: 0.8 },"
echo "        t: Date.now()"
echo "      }));"
echo ""
echo "   4. 在此终端应该看到 ROS2 消息！"
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "⏳ 等待消息（60秒超时）..."
echo ""

# 等待监听进程
wait $MONITOR_PID 2>/dev/null

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🛑 停止服务..."

kill $SIGNALING_PID $EXCAVATOR_PID 2>/dev/null || true

echo "✅ 测试完成"
echo ""
echo "💡 提示:"
echo "   - 查看完整日志: tail -f /tmp/excavator.log"
echo "   - 手动监听话题: ros2 topic echo /controls/teleop std_msgs/msg/String"
echo ""

