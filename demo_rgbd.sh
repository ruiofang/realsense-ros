#!/bin/bash

# RGB-D 融合显示器演示脚本

echo "🎯 RealSense RGB-D 融合显示器演示"
echo "================================="
echo ""

# 检查相机是否运行
if ! pgrep -f realsense > /dev/null; then
    echo "⚠️  未检测到运行中的 RealSense 节点"
    echo "正在启动相机..."
    echo ""
    
    # 启动相机
    cd /home/ruio/realsense-ros
    source /opt/ros/humble/setup.bash
    source install/local_setup.bash
    
    # 后台启动相机
    ros2 launch realsense2_camera rs_launch.py &
    CAMERA_PID=$!
    
    echo "等待相机初始化..."
    sleep 8
    
    # 检查相机是否成功启动
    if ! pgrep -f realsense > /dev/null; then
        echo "❌ 相机启动失败"
        exit 1
    fi
    
    echo "✅ 相机启动成功"
    echo ""
fi

echo "🚀 启动 RGB-D 融合显示器..."
echo ""
echo "💡 功能演示："
echo "   1. 🖱️  鼠标移动 - 实时距离预览"
echo "   2. 🖱️  左键点击 - 添加测量点"
echo "   3. 🖱️  右键点击 - 切换显示模式"
echo "   4. ⌨️  键盘控制："
echo "      - 空格键：切换显示模式"
echo "      - g：显示/隐藏网格"
echo "      - c：显示/隐藏十字准线"
echo "      - u：切换距离单位 (m/cm)"
echo "      - s：保存当前帧和测量数据"
echo "      - r：清除所有测量点"
echo "      - h：显示帮助"
echo "      - q：退出"
echo ""
echo "🎨 显示模式："
echo "   - color：彩色图像 + 距离信息"
echo "   - depth：深度图像可视化"
echo "   - overlay：彩色+深度叠加"
echo ""

# 启动融合显示器
python3 examples/rgbd_fusion.py

echo ""
echo "👋 演示结束"

# 如果我们启动了相机，询问是否关闭
if [ ! -z "$CAMERA_PID" ]; then
    echo ""
    read -p "是否关闭相机节点? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "正在关闭相机..."
        kill $CAMERA_PID 2>/dev/null
        sleep 2
        pkill -f realsense 2>/dev/null
        echo "✅ 相机已关闭"
    fi
fi