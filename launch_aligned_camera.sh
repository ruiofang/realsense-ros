#!/bin/bash

echo "🚀 启动 RealSense 相机 (启用深度对齐功能)"
echo "📝 配置参数:"
echo "   - 深度对齐到彩色图像: 启用"
echo "   - 分辨率: 彩色 1280x720, 深度 848x480"
echo "   - 帧率: 30 FPS"
echo ""

# 启动 RealSense 相机节点，启用深度对齐
ros2 launch realsense2_camera launch_from_rosbag_launch.py \
    device_type:=d400 \
    align_depth.enable:=true \
    pointcloud.enable:=true \
    rgb_camera.color_profile:=1280x720x30 \
    depth_module.depth_profile:=848x480x30 \
    enable_sync:=true