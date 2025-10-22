#!/bin/bash

echo "============================================="
echo "RealSense 相机启动和图像查看指南"
echo "============================================="

# 设置环境
cd /home/ruio/realsense-ros
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch realsense2_camera rs_launch.py

echo -e "\n1. 启动相机节点的命令："
echo "ros2 launch realsense2_camera rs_launch.py"

echo -e "\n2. 启动相机节点（使用自定义参数）："
echo "ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30"

echo -e "\n3. 查看可用话题："
echo "ros2 topic list"

echo -e "\n4. 查看彩色图像话题信息："
echo "ros2 topic info /camera/camera/color/image_raw"

echo -e "\n5. 查看图像消息数据："
echo "ros2 topic echo /camera/camera/color/image_raw --once"

echo -e "\n6. 使用 rviz2 可视化图像："
echo "rviz2"

echo -e "\n7. 在 rviz2 中："
echo "   - 点击 'Add' 按钮"
echo "   - 选择 'Image' 显示类型"
echo "   - 在 Topic 中选择 '/camera/camera/color/image_raw'"

echo -e "\n8. 使用 rqt_image_view 查看图像："
echo "ros2 run rqt_image_view rqt_image_view"

echo -e "\n9. 录制相机数据："
echo "ros2 bag record /camera/camera/color/image_raw /camera/camera/depth/image_rect_raw"

echo -e "\n============================================="
echo "主要的相机话题："
echo "  📷 /camera/camera/color/image_raw - 彩色图像"
echo "  📏 /camera/camera/depth/image_rect_raw - 深度图像"
echo "  ℹ️  /camera/camera/color/camera_info - 彩色相机信息"
echo "  ℹ️  /camera/camera/depth/camera_info - 深度相机信息"
echo "  🔗 /camera/camera/aligned_depth_to_color/image_raw - 对齐的深度图像"
echo "============================================="