#!/usr/bin/env python3
"""
RealSense 深度图像处理示例
订阅并处理 RealSense 相机的深度图像，提供距离测量功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import sys
import os

class DepthProcessor(Node):
    """深度图像处理器类"""
    
    def __init__(self):
        super().__init__('realsense_depth_processor')
        
        # 检查 GUI 支持
        self.gui_available = self.check_gui_support()
        
        # 订阅深度图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.frame_count = 0
        
        # 鼠标回调变量
        self.mouse_x = 0
        self.mouse_y = 0
        self.current_depth_image = None
        
        # 统计变量
        self.depth_stats = {'min': 0, 'max': 0, 'avg': 0, 'valid_pixels': 0}
        
        self.get_logger().info('🚀 RealSense 深度处理器已启动')
        self.get_logger().info('📏 订阅话题: /camera/camera/depth/image_rect_raw')
        
        if self.gui_available:
            self.get_logger().info('🖱️  GUI可用: 在深度图像上移动鼠标查看距离信息')
        else:
            self.get_logger().info('📊 无GUI模式: 将输出深度统计信息到终端')
            
    def check_gui_support(self):
        """检查 GUI 支持"""
        try:
            # 检查是否有 DISPLAY 环境变量
            if 'DISPLAY' not in os.environ:
                return False
                
            # 尝试创建窗口
            cv2.namedWindow('test_window', cv2.WINDOW_NORMAL)
            cv2.destroyWindow('test_window')
            return True
        except Exception:
            return False

    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数"""
        self.mouse_x = x
        self.mouse_y = y
        
        if event == cv2.EVENT_LBUTTONDOWN and self.current_depth_image is not None:
            # 获取点击点的深度值
            depth_value = self.current_depth_image[y, x]
            if depth_value > 0:
                distance = depth_value / 1000.0  # 转换为米
                self.get_logger().info(f'📍 点击位置 ({x}, {y}) 的距离: {distance:.3f}m')
            else:
                self.get_logger().info(f'📍 点击位置 ({x}, {y}) 无有效深度数据')

    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            # 转换深度图像 (16位单通道)
            depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            self.current_depth_image = depth_image
            self.frame_count += 1
            
            # 获取图像尺寸
            height, width = depth_image.shape
            
            # 计算整体深度统计
            valid_depths = depth_image[depth_image > 0]
            
            if len(valid_depths) > 0:
                self.depth_stats = {
                    'min': np.min(valid_depths) / 1000.0,
                    'max': np.max(valid_depths) / 1000.0,
                    'avg': np.mean(valid_depths) / 1000.0,
                    'valid_pixels': len(valid_depths),
                    'total_pixels': height * width
                }
            
            # 计算中心区域的平均距离
            center_x, center_y = width // 2, height // 2
            roi_size = 50  # ROI 大小
            
            x1 = max(0, center_x - roi_size)
            y1 = max(0, center_y - roi_size)
            x2 = min(width, center_x + roi_size)
            y2 = min(height, center_y + roi_size)
            
            center_region = depth_image[y1:y2, x1:x2]
            center_valid_depths = center_region[center_region > 0]
            
            center_avg_distance = 0
            if len(center_valid_depths) > 0:
                center_avg_distance = np.mean(center_valid_depths) / 1000.0
            
            if self.gui_available:
                # GUI 模式 - 显示可视化图像
                self.display_gui(depth_image, center_avg_distance, height, width)
            else:
                # 无GUI模式 - 输出到终端
                self.display_console(center_avg_distance)
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'❌ 深度处理错误: {e}')
            self.get_logger().error(f'详细错误: {traceback.format_exc()}')
    
    def display_gui(self, depth_image, center_avg_distance, height, width):
        """在GUI模式下显示图像"""
        try:
            # 创建深度可视化图像
            depth_normalized = cv2.convertScaleAbs(depth_image, alpha=0.03)
            depth_colormap = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # 计算中心区域框
            center_x, center_y = width // 2, height // 2
            roi_size = 50
            x1 = max(0, center_x - roi_size)
            y1 = max(0, center_y - roi_size)
            x2 = min(width, center_x + roi_size)
            y2 = min(height, center_y + roi_size)
            
            # 绘制中心区域框
            cv2.rectangle(depth_colormap, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 添加文本信息
            info_lines = [
                f"Frame: {self.frame_count}",
                f"Size: {width}x{height}",
                f"Valid Pixels: {self.depth_stats['valid_pixels']}/{self.depth_stats['total_pixels']}",
                f"Depth Range: {self.depth_stats['min']:.3f}-{self.depth_stats['max']:.3f}m",
                f"Average: {self.depth_stats['avg']:.3f}m",
                f"Center Avg: {center_avg_distance:.3f}m"
            ]
            
            # 鼠标位置的深度值
            if 0 <= self.mouse_y < height and 0 <= self.mouse_x < width:
                mouse_depth = depth_image[self.mouse_y, self.mouse_x]
                if mouse_depth > 0:
                    mouse_distance = mouse_depth / 1000.0
                    info_lines.append(f"Mouse ({self.mouse_x},{self.mouse_y}): {mouse_distance:.3f}m")
                else:
                    info_lines.append(f"Mouse ({self.mouse_x},{self.mouse_y}): N/A")
            
            # 在图像上绘制信息
            for i, line in enumerate(info_lines):
                y_pos = 25 + i * 25
                cv2.putText(depth_colormap, line, (10, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 在鼠标位置绘制十字线
            if self.mouse_x > 0 and self.mouse_y > 0:
                cv2.line(depth_colormap, (self.mouse_x - 10, self.mouse_y), 
                        (self.mouse_x + 10, self.mouse_y), (0, 255, 255), 2)
                cv2.line(depth_colormap, (self.mouse_x, self.mouse_y - 10), 
                        (self.mouse_x, self.mouse_y + 10), (0, 255, 255), 2)
            
            # 显示深度图像
            cv2.imshow("RealSense 深度图像", depth_colormap)
            cv2.setMouseCallback("RealSense 深度图像", self.mouse_callback)
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('👋 用户退出程序')
                rclpy.shutdown()
            elif key == ord('s'):
                # 保存深度图像和可视化图像
                cv2.imwrite(f"depth_raw_{self.frame_count:04d}.png", depth_image)
                cv2.imwrite(f"depth_colormap_{self.frame_count:04d}.jpg", depth_colormap)
                self.get_logger().info(f'💾 已保存深度图像: depth_*_{self.frame_count:04d}.*')
            elif key == ord('h'):
                # 显示帮助信息
                self.print_help()
                
        except Exception as e:
            self.get_logger().error(f'❌ GUI显示错误: {e}')
            # 如果GUI出错，切换到无GUI模式
            self.gui_available = False
            self.get_logger().info('🔄 切换到无GUI模式')
            
    def display_console(self, center_avg_distance):
        """在无GUI模式下输出到终端"""
        # 每30帧输出一次统计信息
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'📊 帧 #{self.frame_count}:')
            self.get_logger().info(f'   📏 深度范围: {self.depth_stats["min"]:.3f}m - {self.depth_stats["max"]:.3f}m')
            self.get_logger().info(f'   📈 平均深度: {self.depth_stats["avg"]:.3f}m')
            self.get_logger().info(f'   🎯 中心区域: {center_avg_distance:.3f}m')
            self.get_logger().info(f'   📍 有效像素: {self.depth_stats["valid_pixels"]}/{self.depth_stats["total_pixels"]} ({100*self.depth_stats["valid_pixels"]/self.depth_stats["total_pixels"]:.1f}%)')
            
        # 每5帧输出简要信息
        elif self.frame_count % 5 == 0:
            self.get_logger().info(f'帧 #{self.frame_count}: 中心距离 {center_avg_distance:.3f}m, 平均 {self.depth_stats["avg"]:.3f}m')

    def print_help(self):
        """打印帮助信息"""
        help_text = """
🔧 深度图像处理器控制帮助:
   q - 退出程序
   s - 保存当前深度图像
   h - 显示此帮助信息
   鼠标移动 - 显示实时深度值
   鼠标左键 - 打印点击位置深度值
   
📊 显示信息说明:
   - 绿色框: 中心区域 ROI
   - 黄色十字: 鼠标位置
   - Center Avg: 中心区域平均距离
   - Mouse Depth: 鼠标位置深度值
        """
        print(help_text)

def main(args=None):
    """主函数"""
    print("🎯 启动 RealSense 深度处理器...")
    print("💡 提示:")
    print("   - 移动鼠标查看实时深度值")
    print("   - 点击鼠标获取精确距离")
    print("   - 按 'h' 键查看完整帮助")
    print("   - 按 'q' 键或 Ctrl+C 退出")
    print()
    
    rclpy.init(args=args)
    
    depth_processor = DepthProcessor()
    
    try:
        rclpy.spin(depth_processor)
    except KeyboardInterrupt:
        print("\n🛑 接收到键盘中断信号")
    except Exception as e:
        print(f"❌ 运行时错误: {e}")
    finally:
        # 清理资源
        if rclpy.ok():
            depth_processor.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()
        print("✅ 程序已正常退出")

if __name__ == '__main__':
    main()