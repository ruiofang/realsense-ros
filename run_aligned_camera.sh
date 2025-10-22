#!/bin/bash

echo "🚀 启动 RealSense 相机 (启用深度对齐功能)"
echo "📝 这将启用深度图像对齐到彩色图像"
echo ""
cd /home/ruio/realsense-ros
source /opt/ros/humble/setup.bash
source install/local_setup.bash
# 启动 RealSense 相机节点，启用深度对齐
ros2 run realsense2_camera realsense2_camera_node \
    --ros-args \
    -p align_depth.enable:=true \
    -p enable_sync:=true \
    -p pointcloud.enable:=true