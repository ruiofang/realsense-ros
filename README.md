# Intel RealSense ROS2 完整使用教程

## 📖 目录

1. [简介](#简介)
2. [环境准备](#环境准备)
3. [安装依赖](#安装依赖)
4. [构建项目](#构建项目)
5. [启动相机](#启动相机)
6. [查看图像](#查看图像)
7. [高级功能](#高级功能)
8. [常见问题](#常见问题)
9. [开发示例](#开发示例)

---

## 🎯 简介

Intel RealSense 是一系列深度感知相机，提供彩色图像、深度信息和IMU数据。本教程将指导你在 ROS2 环境中完整使用 RealSense 相机。

### 支持的相机型号

- D400 系列（D415, D435, D435i, D455, D456等）
- L515（激光雷达相机）
- SR300 系列

---

## 🛠️ 环境准备

### 系统要求

- Ubuntu 20.04/22.04/24.04
- ROS2 Humble/Iron/Jazzy/Rolling
- USB 3.0 接口

### 检查 ROS2 环境

```bash
# 检查 ROS2 版本
echo $ROS_DISTRO

# 确认环境变量
env | grep ROS
```

---

## 📦 安装依赖

### 1. 安装 Intel RealSense SDK

```bash
# 添加 Intel 仓库密钥
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# 添加仓库
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# 安装 librealsense2
sudo apt-get install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg -y
```

### 2. 验证安装

```bash
# 检查连接的相机
rs-enumerate-devices

# 应该看到类似输出：
# Device info: 
#     Name                          :     Intel RealSense D456
#     Serial Number                 :     339522300042
#     Firmware Version              :     5.15.0.2
```

### 3. 安装 ROS2 依赖

```bash
# 安装构建工具
sudo apt-get install python3-rosdep python3-colcon-common-extensions -y

# 初始化 rosdep（如果未初始化）
sudo rosdep init
rosdep update
```

---

## 🔨 构建项目

### 1. 获取源码

```bash
# 创建工作空间
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/

# 克隆 RealSense ROS2 包
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ~/ros2_ws
```

### 2. 安装依赖项

```bash
# 安装 ROS 包依赖
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y
```

### 3. 构建项目

```bash
# 构建所有包
colcon build

# 或者构建特定包
colcon build --packages-select realsense2_camera
```

### 4. 设置环境

```bash
# 设置环境变量
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash
```

---

## 🚀 启动相机

### 基本启动

```bash
# 方法1：使用 launch 文件（推荐）
ros2 launch realsense2_camera rs_launch.py

# 方法2：直接运行节点
ros2 run realsense2_camera realsense2_camera_node
```

### 自定义参数启动

```bash
# 设置分辨率和帧率
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30

# 启用点云
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true

# 启用 IMU
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true \
  enable_accel:=true
```

---

## 👁️ 查看图像

### 1. 检查话题

```bash
# 查看所有话题
ros2 topic list

# 查看相机话题
ros2 topic list | grep camera

# 主要话题：
# /camera/camera/color/image_raw        - 彩色图像
# /camera/camera/depth/image_rect_raw   - 深度图像
# /camera/camera/color/camera_info      - 相机参数
```

### 2. 使用 rviz2 可视化

#### 启动 rviz2

```bash
rviz2
```

#### 配置显示

1. **添加图像显示**：

   - 点击 `Add` → `By display type` → `Image`
   - 在 Topic 中选择 `/camera/camera/color/image_raw`
2. **添加点云显示**（如果启用）：

   - 点击 `Add` → `By display type` → `PointCloud2`
   - 在 Topic 中选择 `/camera/camera/depth/color/points`
3. **设置坐标系**：

   - 将 Fixed Frame 设置为 `camera_link`

### 3. 使用 rqt_image_view

```bash
# 启动图像查看器
ros2 run rqt_image_view rqt_image_view

# 在界面中选择相应的图像话题
```

---

## 🔧 高级功能

### 1. 参数配置

#### 查看参数

```bash
# 列出所有参数
ros2 param list

# 查看特定参数
ros2 param get /camera/camera depth_module.depth_profile
```

#### 运行时修改参数

```bash
# 修改曝光时间
ros2 param set /camera/camera depth_module.exposure 8500

# 启用/禁用流
ros2 param set /camera/camera enable_depth true
ros2 param set /camera/camera enable_color false
```

### 2. 录制数据

```bash
# 录制彩色和深度图像
ros2 bag record \
  /camera/camera/color/image_raw \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/color/camera_info \
  /camera/camera/depth/camera_info

# 录制所有相机话题
ros2 bag record -a --regex ".*camera.*"
```

### 3. 回放数据

```bash
# 回放录制的数据
ros2 bag play <bag_file_name>
```

---

## 📚 故障排除

### 常见问题解决方案

#### 1. OpenCV GUI 显示问题

**错误信息：**

```
OpenCV(4.11.0) /io/opencv/modules/highgui/src/window.cpp:1301: error: (-2:Unspecified error) The function is not implemented. Rebuild the library with Windows, GTK+ 2.x or Cocoa support.
```

**解决方案：**

```bash
# 安装 GUI 支持库
sudo apt update
sudo apt install -y libgtk2.0-dev pkg-config libgtk-3-dev libcanberra-gtk-module libcanberra-gtk3-module

# 卸载 pip 版本的 OpenCV，安装系统版本
pip3 uninstall -y opencv-python opencv-contrib-python
sudo apt install -y python3-opencv

# 测试 GUI 支持
python3 -c "import cv2; cv2.namedWindow('test'); cv2.destroyAllWindows(); print('GUI支持正常')"
```

#### 2. 相机无法检测

```bash
# 检查 USB 连接
lsusb | grep Intel

# 检查权限
ls -la /dev/video*

# 重新插拔 USB 线缆
```

### 2. "Device or resource busy" 错误

```bash
# 终止所有相关进程
pkill -f realsense

# 等待几秒后重新启动
sleep 3
ros2 launch realsense2_camera rs_launch.py
```

### 3. 帧率太低或图像卡顿

```bash
# 使用较低分辨率
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=424x240x30 \
  rgb_camera.color_profile:=424x240x30

# 检查 USB 连接（确保使用 USB 3.0）
```

### 4. IMU 数据不可用

```bash
# 启用 IMU 流
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true \
  enable_accel:=true \
  enable_sync:=true
```

---

## �️ 自定义工具集

本项目包含了多个专门开发的可视化和处理工具，提供更好的用户体验：

### 1. **RGB-D 融合显示器** ⭐推荐⭐

文件：`examples/rgbd_fusion.py`

**特点：**

- 🎨 在直观的彩色图像上实时显示距离信息
- 🖱️ 鼠标移动实时预览距离值
- 📍 左键点击添加测量点，右键切换显示模式
- 🔄 三种显示模式：彩色、深度、叠加
- 📊 可选网格、十字准线等可视化辅助
- 💾 保存图像和测量数据
- ⚙️ 距离单位切换 (米/厘米)

**启动：**

```bash
python3 examples/rgbd_fusion.py
```

**控制：**

- 鼠标移动：实时距离预览
- 左键：添加测量点
- 右键/空格：切换显示模式
- g：网格显示切换
- c：十字准线切换
- u：距离单位切换
- s：保存当前帧
- r：清除测量点
- q：退出

### 2. 彩色图像查看器

文件：`examples/color_viewer.py`

- 实时彩色图像显示
- FPS 和图像信息显示
- 全屏模式切换
- 图像保存功能

### 3. 深度图像处理器

文件：`examples/depth_processor.py`

- 交互式深度测量
- 彩色深度图可视化
- 鼠标点击精确测距
- 深度统计信息

### 4. 高级图像处理器

文件：`examples/advanced_processor.py`

- 多摄像头数据同时显示
- 视频录制功能
- 实时统计信息
- 组合视图显示

### 快速启动

```bash
# RGB-D 融合演示
./demo_rgbd.sh

# 系统状态检查
./test_tools.sh
```

---

## �💻 开发示例

### 1. 订阅彩色图像

创建 `image_subscriber.py`：

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
    
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
    
        # CV Bridge 用于图像转换
        self.bridge = CvBridge()
    
        self.get_logger().info('图像订阅者已启动')

    def image_callback(self, msg):
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
            # 显示图像
            cv2.imshow("RealSense 彩色图像", cv_image)
            cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')

def main(args=None):
    rclpy.init(args=args)
  
    image_subscriber = ImageSubscriber()
  
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
  
    image_subscriber.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### 2. 处理深度图像

创建 `depth_processor.py`：

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')
    
        # 订阅深度图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
    
        self.bridge = CvBridge()
        self.get_logger().info('深度处理器已启动')

    def depth_callback(self, msg):
        try:
            # 转换深度图像
            depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        
            # 转换为可视化格式
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03),
                cv2.COLORMAP_JET
            )
        
            # 显示深度图像
            cv2.imshow("RealSense 深度图像", depth_colormap)
            cv2.waitKey(1)
        
            # 计算平均距离（中心区域）
            h, w = depth_image.shape
            center_region = depth_image[h//4:3*h//4, w//4:3*w//4]
            valid_depths = center_region[center_region > 0]
        
            if len(valid_depths) > 0:
                avg_distance = np.mean(valid_depths) / 1000.0  # 转换为米
                self.get_logger().info(f'中心区域平均距离: {avg_distance:.2f}m')
        
        except Exception as e:
            self.get_logger().error(f'深度处理错误: {e}')

def main(args=None):
    rclpy.init(args=args)
  
    depth_processor = DepthProcessor()
  
    try:
        rclpy.spin(depth_processor)
    except KeyboardInterrupt:
        pass
  
    depth_processor.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### 3. 启动脚本

创建便捷启动脚本 `start_realsense.sh`：

```bash
#!/bin/bash

echo "🚀 启动 Intel RealSense ROS2 相机..."

# 进入工作目录
cd ~/ros2_ws

# 设置环境
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

# 清理之前的进程
pkill -f realsense 2>/dev/null || true
sleep 2

# 启动相机
echo "📷 启动相机节点..."
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x30 \
  rgb_camera.color_profile:=640x480x30 \
  pointcloud.enable:=true \
  align_depth.enable:=true &

# 等待启动
echo "⏳ 等待相机初始化..."
sleep 8

# 检查话题
echo "📋 可用话题："
ros2 topic list | grep camera

echo "✅ RealSense 相机已启动！"
echo "💡 使用以下命令查看图像："
echo "   rviz2"
echo "   ros2 run rqt_image_view rqt_image_view"

# 保持脚本运行
wait
```

### 4. 运行示例

```bash
# 给脚本执行权限
chmod +x start_realsense.sh

# 启动相机
./start_realsense.sh

# 在新终端运行Python示例
cd ~/ros2_ws
source install/local_setup.bash
python3 image_subscriber.py
```

---

## 📚 参考资源

- [Intel RealSense 官方文档](https://intelrealsense.github.io/librealsense/)
- [ROS2 RealSense 包文档](https://github.com/IntelRealSense/realsense-ros)
- [ROS2 官方教程](https://docs.ros.org/en/humble/Tutorials.html)

---

## 🤝 贡献

欢迎提交问题和改进建议！

---

**© 2024 Intel RealSense ROS2 教程 | 最后更新: 2024年10月22日**
