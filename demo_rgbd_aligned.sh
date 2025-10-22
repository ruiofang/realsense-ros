#!/bin/bash

echo "🎯 RealSense RGB-D 融合显示演示"
echo "================================="
echo ""
echo "📖 使用说明："
echo ""
echo "1️⃣ 首先启动相机（启用深度对齐）："
echo "   ./start_aligned_camera.sh"
echo ""
echo "2️⃣ 在新终端中运行RGB-D融合显示器："
echo "   python3 examples/rgbd_fusion.py"
echo ""
echo "💡 如果深度图像和彩色图像没有对齐，请确保："
echo "   - 使用了 align_depth.enable:=true 参数启动相机"
echo "   - 两个图像的分辨率匹配"
echo ""
echo "🔧 当前修改："
echo "   - 自动调整深度图像尺寸以匹配彩色图像"
echo "   - 坐标映射以正确获取深度值"
echo "   - 支持不同分辨率的深度和彩色图像"
echo ""
echo "🎮 控制说明："
echo "   鼠标移动: 实时显示距离"
echo "   左键点击: 测量并记录距离"
echo "   右键点击: 切换显示模式"
echo "   空格键: 切换显示模式"
echo "   'q' 键: 退出"
echo "   'h' 键: 显示帮助"
echo ""

read -p "按回车键继续..."

echo ""
echo "🚀 现在开始演示..."
echo ""

# 检查相机是否正在运行
if pgrep -f "realsense2_camera_node" > /dev/null; then
    echo "✅ 检测到 RealSense 相机正在运行"
else
    echo "⚠️  未检测到 RealSense 相机，正在启动..."
    gnome-terminal -- bash -c "./start_aligned_camera.sh; exec bash"
    sleep 3
fi

echo "🎬 启动 RGB-D 融合显示器..."
python3 examples/rgbd_fusion.py