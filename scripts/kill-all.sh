#!/bin/bash

# 停止所有相关进程

echo "🛑 停止所有 excavator 相关进程..."

pkill -f "signaling -addr" && echo "  ✅ 已停止 signaling 服务器" || echo "  ℹ️  signaling 服务器未运行"
pkill -f "excavator -signaling" && echo "  ✅ 已停止 excavator 程序" || echo "  ℹ️  excavator 程序未运行"

echo "✅ 完成"

