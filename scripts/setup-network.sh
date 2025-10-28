#!/bin/bash

# 网络配置脚本 - 自动配置 IP 地址和防火墙

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

# 获取本机 IP
LOCAL_IP=$(hostname -I | awk '{print $1}')

if [ -z "$LOCAL_IP" ]; then
    echo "❌ 无法获取本机 IP 地址"
    exit 1
fi

echo "🔧 网络配置工具"
echo ""
echo "📍 检测到本机 IP: $LOCAL_IP"
echo ""

# 1. 修改 controller.html 中的信令服务器地址
echo "1️⃣ 修改控制页面配置..."

CONTROLLER_FILE="$PROJECT_ROOT/web/controller.html"
BACKUP_FILE="$PROJECT_ROOT/web/controller.html.bak"

# 备份原文件
if [ ! -f "$BACKUP_FILE" ]; then
    cp "$CONTROLLER_FILE" "$BACKUP_FILE"
    echo "   ✅ 已备份原始文件: controller.html.bak"
fi

# 替换信令服务器地址
sed -i "s|const SIGNALING_SERVER = 'ws://[^']*';|const SIGNALING_SERVER = 'ws://$LOCAL_IP:8090/ws';|g" "$CONTROLLER_FILE"
echo "   ✅ 已更新信令服务器地址: ws://$LOCAL_IP:8090/ws"

# 2. 检查防火墙
echo ""
echo "2️⃣ 检查防火墙配置..."

# 检查 ufw 是否安装
if command -v ufw >/dev/null 2>&1; then
    UFW_STATUS=$(sudo ufw status | grep -i "Status:" | awk '{print $2}')
    
    if [ "$UFW_STATUS" = "active" ]; then
        echo "   ℹ️  防火墙已启用"
        
        # 检查端口是否开放
        if ! sudo ufw status | grep -q "8090.*ALLOW"; then
            echo "   ⚠️  端口 8090 未开放，正在开放..."
            sudo ufw allow 8090/tcp
            echo "   ✅ 已开放端口 8090 (信令服务器)"
        else
            echo "   ✅ 端口 8090 已开放"
        fi
        
        if ! sudo ufw status | grep -q "8080.*ALLOW"; then
            echo "   ⚠️  端口 8080 未开放，正在开放..."
            sudo ufw allow 8080/tcp
            echo "   ✅ 已开放端口 8080 (Web 服务器)"
        else
            echo "   ✅ 端口 8080 已开放"
        fi
    else
        echo "   ✅ 防火墙未启用，无需配置"
    fi
else
    echo "   ℹ️  未检测到 ufw 防火墙"
fi

# 3. 修改 start-excavator.sh 中的信令服务器地址
echo ""
echo "3️⃣ 修改挖掘机脚本配置..."

EXCAVATOR_SCRIPT="$PROJECT_ROOT/scripts/start-excavator.sh"
sed -i "s|SIGNALING_SERVER=\"ws://[^\"]*\"|SIGNALING_SERVER=\"ws://$LOCAL_IP:8090/ws\"|g" "$EXCAVATOR_SCRIPT"
echo "   ✅ 已更新挖掘机脚本"

echo ""
echo "✅ 网络配置完成！"
echo ""
echo "📋 配置摘要:"
echo "   - 本机 IP: $LOCAL_IP"
echo "   - 信令服务器: ws://$LOCAL_IP:8090/ws"
echo "   - Web 服务器: http://$LOCAL_IP:8080/controller.html"
echo ""
echo "🚀 下一步:"
echo "   1. 在 Orin 上启动信令服务器:"
echo "      ./scripts/start-signaling.sh"
echo ""
echo "   2. 在 Orin 上启动挖掘机程序:"
echo "      ./scripts/start-excavator.sh"
echo ""
echo "   3. 在 Orin 上启动 Web 服务器:"
echo "      ./scripts/start-web-server.sh"
echo ""
echo "   4. 在其他电脑浏览器访问:"
echo "      http://$LOCAL_IP:8080/controller.html"
echo ""

