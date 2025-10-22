#!/usr/bin/env python3
"""
RealSense ROS2 图像处理工具包
提供多种图像处理和分析功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import threading
from pathlib import Path
import json

class ImageProcessorNode(Node):
    """高级图像处理节点"""
    
    def __init__(self):
        super().__init__('image_processor')
        
        # 参数
        self.declare_parameter('save_path', str(Path.home() / 'realsense_captures'))
        self.declare_parameter('show_fps', True)
        self.declare_parameter('enable_recording', False)
        self.declare_parameter('max_fps', 30)
        
        # 状态变量
        self.bridge = CvBridge()
        self.last_time = time.time()
        self.frame_count = 0
        self.fps = 0.0
        self.save_path = Path(self.get_parameter('save_path').value)
        self.save_path.mkdir(exist_ok=True)
        
        # 图像缓存
        self.color_image = None
        self.depth_image = None
        self.infrared_image = None
        
        # 录制相关
        self.video_writer = None
        self.recording = False
        
        # 创建订阅者
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', 
            self.color_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', 
            self.depth_callback, 10)
        
        self.infrared_sub = self.create_subscription(
            Image, '/camera/camera/infra1/image_rect_raw', 
            self.infrared_callback, 10)
        
        # 创建发布者（处理后的图像）
        self.processed_pub = self.create_publisher(
            Image, '/camera/processed/color', 10)
        
        self.overlay_pub = self.create_publisher(
            Image, '/camera/processed/overlay', 10)
        
        # 定时器
        self.timer = self.create_timer(0.1, self.process_images)
        
        # 初始化 UI
        self.init_ui()
        
        self.get_logger().info("🎨 RealSense 图像处理器已启动")
        self.get_logger().info(f"📁 保存路径: {self.save_path}")
    
    def init_ui(self):
        """初始化用户界面"""
        cv2.namedWindow('RealSense Control', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('RealSense Control', 400, 300)
        
        # 创建控制面板
        control_img = np.zeros((300, 400, 3), dtype=np.uint8)
        cv2.putText(control_img, 'RealSense Control Panel', (50, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        cv2.putText(control_img, 'Keys:', (20, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(control_img, 's: Save current frame', (20, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(control_img, 'r: Start/Stop recording', (20, 120), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(control_img, 'f: Toggle FPS display', (20, 140), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(control_img, 'h: Show/Hide help', (20, 160), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(control_img, 'q: Quit', (20, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        cv2.putText(control_img, f'Status: Ready', (20, 220), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        cv2.imshow('RealSense Control', control_img)
    
    def color_callback(self, msg):
        """彩色图像回调"""
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.update_fps()
        except Exception as e:
            self.get_logger().error(f"彩色图像转换错误: {e}")
    
    def depth_callback(self, msg):
        """深度图像回调"""
        try:
            # 深度图像通常是 16-bit
            depth_raw = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            # 转换为可视化格式
            depth_normalized = cv2.normalize(depth_raw, None, 0, 255, cv2.NORM_MINMAX)
            self.depth_image = depth_normalized.astype(np.uint8)
        except Exception as e:
            self.get_logger().error(f"深度图像转换错误: {e}")
    
    def infrared_callback(self, msg):
        """红外图像回调"""
        try:
            self.infrared_image = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        except Exception as e:
            self.get_logger().error(f"红外图像转换错误: {e}")
    
    def update_fps(self):
        """更新 FPS 计算"""
        current_time = time.time()
        self.frame_count += 1
        
        if current_time - self.last_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_time)
            self.frame_count = 0
            self.last_time = current_time
    
    def process_images(self):
        """处理图像并显示"""
        if self.color_image is None:
            return
        
        # 处理按键
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            self.cleanup()
            rclpy.shutdown()
            return
        elif key == ord('s'):
            self.save_frame()
        elif key == ord('r'):
            self.toggle_recording()
        elif key == ord('f'):
            show_fps = not self.get_parameter('show_fps').value
            self.set_parameters([rclpy.Parameter('show_fps', value=show_fps)])
        elif key == ord('h'):
            self.show_help()
        
        # 显示图像
        self.display_images()
        
        # 录制视频
        if self.recording and self.video_writer is not None:
            self.video_writer.write(self.color_image)
    
    def display_images(self):
        """显示所有图像"""
        display_image = self.color_image.copy()
        
        # 添加 FPS 显示
        if self.get_parameter('show_fps').value:
            fps_text = f'FPS: {self.fps:.1f}'
            cv2.putText(display_image, fps_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        
        # 添加录制状态
        if self.recording:
            cv2.circle(display_image, (display_image.shape[1] - 30, 30), 
                      10, (0, 0, 255), -1)
            cv2.putText(display_image, 'REC', 
                       (display_image.shape[1] - 70, 35), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # 显示彩色图像
        cv2.imshow('Color Image', display_image)
        
        # 显示深度图像（如果可用）
        if self.depth_image is not None:
            # 应用颜色映射
            depth_colormap = cv2.applyColorMap(self.depth_image, cv2.COLORMAP_JET)
            cv2.imshow('Depth Image', depth_colormap)
        
        # 显示红外图像（如果可用）
        if self.infrared_image is not None:
            cv2.imshow('Infrared Image', self.infrared_image)
        
        # 创建组合视图
        if self.depth_image is not None:
            self.create_combined_view(display_image, self.depth_image)
        
        # 发布处理后的图像
        self.publish_processed_image(display_image)
    
    def create_combined_view(self, color_img, depth_img):
        """创建组合视图"""
        # 调整尺寸
        h, w = color_img.shape[:2]
        depth_resized = cv2.resize(depth_img, (w, h))
        depth_colored = cv2.applyColorMap(depth_resized, cv2.COLORMAP_JET)
        
        # 水平拼接
        combined = np.hstack((color_img, depth_colored))
        
        # 添加标签
        cv2.putText(combined, 'Color', (10, h - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(combined, 'Depth', (w + 10, h - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow('Combined View', combined)
    
    def publish_processed_image(self, processed_img):
        """发布处理后的图像"""
        try:
            # 转换为 ROS 消息
            img_msg = self.bridge.cv2_to_imgmsg(processed_img, 'bgr8')
            img_msg.header = Header()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_color_optical_frame'
            
            # 发布
            self.processed_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"发布图像错误: {e}")
    
    def save_frame(self):
        """保存当前帧"""
        if self.color_image is None:
            return
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        # 保存彩色图像
        color_path = self.save_path / f"color_{timestamp}.jpg"
        cv2.imwrite(str(color_path), self.color_image)
        
        # 保存深度图像（如果可用）
        if self.depth_image is not None:
            depth_path = self.save_path / f"depth_{timestamp}.png"
            cv2.imwrite(str(depth_path), self.depth_image)
        
        # 保存红外图像（如果可用）
        if self.infrared_image is not None:
            ir_path = self.save_path / f"infrared_{timestamp}.png"
            cv2.imwrite(str(ir_path), self.infrared_image)
        
        # 保存元数据
        metadata = {
            'timestamp': timestamp,
            'fps': self.fps,
            'color_image': str(color_path),
            'depth_image': str(depth_path) if self.depth_image is not None else None,
            'infrared_image': str(ir_path) if self.infrared_image is not None else None,
            'image_size': self.color_image.shape[:2]
        }
        
        metadata_path = self.save_path / f"metadata_{timestamp}.json"
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        self.get_logger().info(f"💾 帧已保存: {color_path}")
    
    def toggle_recording(self):
        """切换录制状态"""
        if not self.recording:
            # 开始录制
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            video_path = self.save_path / f"recording_{timestamp}.mp4"
            
            if self.color_image is not None:
                h, w, _ = self.color_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(
                    str(video_path), fourcc, 30.0, (w, h))
                
                self.recording = True
                self.get_logger().info(f"🎬 开始录制: {video_path}")
        else:
            # 停止录制
            self.recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            self.get_logger().info("⏹️ 录制已停止")
    
    def show_help(self):
        """显示帮助信息"""
        help_text = """
        RealSense 图像处理器 - 帮助

        键盘快捷键:
        s - 保存当前帧（所有类型）
        r - 开始/停止视频录制
        f - 切换 FPS 显示
        h - 显示/隐藏此帮助
        q - 退出程序

        窗口说明:
        - Color Image: 彩色摄像头图像
        - Depth Image: 深度图像（彩色映射）
        - Infrared Image: 红外图像
        - Combined View: 彩色和深度组合视图
        - RealSense Control: 控制面板

        保存文件位置: {}
        """.format(self.save_path)
        
        print(help_text)
        self.get_logger().info("📋 帮助信息已显示在终端")
    
    def cleanup(self):
        """清理资源"""
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()


def main():
    """主函数"""
    print("🎨 启动 RealSense 图像处理器...")
    
    # 初始化 ROS2
    rclpy.init()
    
    try:
        # 创建节点
        processor = ImageProcessorNode()
        
        # 运行节点
        rclpy.spin(processor)
        
    except KeyboardInterrupt:
        print("\n👋 用户中断，正在退出...")
    
    except Exception as e:
        print(f"❌ 错误: {e}")
    
    finally:
        # 清理
        if 'processor' in locals():
            processor.cleanup()
        rclpy.shutdown()
        print("✅ 清理完成")


if __name__ == '__main__':
    main()