#!/bin/bash

echo "🚀 启动 RealSense 相机..."

# 进入工作目录
cd /home/ruio/realsense-ros

# 设置环境
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source install/local_setup.bash

# 启动相机节点
echo "正在启动相机节点..."
ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30 &

# 记录进程ID
CAMERA_PID=$!
echo "相机节点已启动，进程ID: $CAMERA_PID"

# 等待节点启动
echo "等待节点初始化..."
sleep 10

# 检查节点状态
echo -e "\n=== 检查节点状态 ==="
ros2 node list

echo -e "\n=== 可用话题 ==="
ros2 topic list | grep camera

echo -e "\n=== 彩色图像话题信息 ==="
ros2 topic info /camera/camera/color/image_raw

echo -e "\n✅ 相机节点运行正常！"
echo "📷 彩色图像话题: /camera/camera/color/image_raw"
echo "📏 深度图像话题: /camera/camera/depth/image_rect_raw"

echo -e "\n要查看图像，请运行:"
echo "  rviz2  (然后添加Image显示，选择相应话题)"
echo "  ros2 run rqt_image_view rqt_image_view"

echo -e "\n按 Ctrl+C 停止相机节点"
wait $CAMERA_PID