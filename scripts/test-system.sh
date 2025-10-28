#!/bin/bash

# 系统测试脚本 - 在本机测试整个系统

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

echo "🧪 开始系统测试..."
echo ""

# 1. 检查编译产物
echo "1️⃣ 检查编译产物..."
if [ ! -f "bin/signaling" ]; then
    echo "❌ 未找到 signaling 可执行文件，请先编译"
    exit 1
fi
if [ ! -f "bin/excavator" ]; then
    echo "❌ 未找到 excavator 可执行文件，请先编译"
    exit 1
fi
echo "✅ 编译产物检查通过"
echo ""

# 2. 检查摄像头
echo "2️⃣ 检查摄像头..."
if [ ! -e "/dev/video0" ]; then
    echo "⚠️  摄像头 /dev/video0 不存在"
    echo "可用设备:"
    ls -l /dev/video* 2>/dev/null || echo "  没有找到摄像头设备"
    echo ""
    read -p "是否继续测试？(y/n): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo "✅ 摄像头检查通过"
fi
echo ""

# 3. 停止旧进程
echo "3️⃣ 清理旧进程..."
pkill -f "signaling -addr" 2>/dev/null || true
pkill -f "excavator -signaling" 2>/dev/null || true
sleep 1
echo "✅ 清理完成"
echo ""

# 4. 启动信令服务器
echo "4️⃣ 启动信令服务器..."
cd bin
./signaling -addr :8090 &
SIGNALING_PID=$!
sleep 2

if ! ps -p $SIGNALING_PID > /dev/null; then
    echo "❌ 信令服务器启动失败"
    exit 1
fi
echo "✅ 信令服务器启动成功 (PID: $SIGNALING_PID)"
echo ""

# 5. 启动挖掘机程序
echo "5️⃣ 启动挖掘机程序..."
./excavator -signaling ws://127.0.0.1:8090/ws -camera /dev/video0 -width 640 -height 480 -fps 30 &
EXCAVATOR_PID=$!
sleep 2

if ! ps -p $EXCAVATOR_PID > /dev/null; then
    echo "❌ 挖掘机程序启动失败"
    kill $SIGNALING_PID 2>/dev/null || true
    exit 1
fi
echo "✅ 挖掘机程序启动成功 (PID: $EXCAVATOR_PID)"
echo ""

# 6. 显示状态
echo "6️⃣ 当前状态："
echo "   信令服务器: http://127.0.0.1:8090/status"
echo "   控制页面: file://$PROJECT_ROOT/web/controller.html"
echo ""
echo "📝 测试步骤："
echo "   1. 打开浏览器访问 controller.html"
echo "   2. 点击 '连接挖掘机' 按钮"
echo "   3. 查看视频画面"
echo ""
echo "💡 按 Ctrl+C 停止所有服务"
echo ""

# 等待用户中断
trap "echo ''; echo '🛑 停止服务...'; kill $SIGNALING_PID $EXCAVATOR_PID 2>/dev/null || true; echo '✅ 已停止'; exit 0" INT TERM

# 保持运行
wait

