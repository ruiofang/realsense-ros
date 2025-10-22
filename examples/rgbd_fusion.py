#!/usr/bin/env python3
"""
RealSense RGB-D 融合显示器
在彩色图像上显示深度信息，提供更直观的距离测量体验
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time
import threading
from collections import deque

class RGBDepthFusion(Node):
    """RGB-D 融合显示器类"""
    
    def __init__(self):
        super().__init__('realsense_rgbd_fusion')
        
        # 检查 GUI 支持
        self.gui_available = self.check_gui_support()
        
        # 订阅彩色和深度图像
        self.color_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10
        )
        
        # 尝试订阅对齐的深度图像，如果不可用则使用普通深度图像
        self.aligned_depth_available = False
        self.depth_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        
        # 首先检查对齐深度话题是否可用
        try:
            import subprocess
            result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
            available_topics = result.stdout.strip().split('\n')
            
            if self.depth_topic in available_topics:
                self.aligned_depth_available = True
                self.get_logger().info('✅ 使用对齐的深度图像')
            else:
                self.depth_topic = '/camera/camera/depth/image_rect_raw'
                self.get_logger().info('⚠️  对齐深度图像不可用，使用普通深度图像')
                self.get_logger().info('💡 要启用对齐功能，请使用: align_depth.enable:=true')
        except Exception:
            self.depth_topic = '/camera/camera/depth/image_rect_raw'
            self.get_logger().info('⚠️  使用普通深度图像')
        
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0
        
        # 图像数据
        self.color_image = None
        self.depth_image = None
        self.data_lock = threading.Lock()
        
        # 鼠标回调变量
        self.mouse_x = 0
        self.mouse_y = 0
        self.mouse_distance = 0.0
        self.click_points = deque(maxlen=10)  # 保存最近10个点击点
        
        # 显示模式
        self.display_mode = 'color'  # 'color', 'depth', 'overlay'
        self.show_grid = False
        self.show_crosshair = True
        self.distance_unit = 'm'  # 'm' 或 'cm'
        
        self.get_logger().info('🚀 RealSense RGB-D 融合显示器已启动')
        self.get_logger().info('📸 订阅彩色话题: /camera/camera/color/image_raw')
        self.get_logger().info(f'📏 订阅深度话题: {self.depth_topic}')
        
        if self.gui_available:
            self.get_logger().info('🖱️  GUI可用: 在彩色图像上显示距离信息')
        else:
            self.get_logger().info('📊 无GUI模式: 将输出统计信息到终端')
            
    def check_gui_support(self):
        """检查 GUI 支持"""
        try:
            if 'DISPLAY' not in os.environ:
                return False
            cv2.namedWindow('test_window', cv2.WINDOW_NORMAL)
            cv2.destroyWindow('test_window')
            return True
        except Exception:
            return False

    def color_callback(self, msg):
        """彩色图像回调函数"""
        try:
            with self.data_lock:
                self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                self.frame_count += 1
                
                # 第一帧时输出调试信息
                if self.frame_count == 1:
                    self.get_logger().info(f'📸 收到第一帧彩色图像: {self.color_image.shape}')
                
                # 计算 FPS
                current_time = time.time()
                if self.frame_count % 30 == 0:
                    elapsed_time = current_time - self.start_time
                    self.fps = 30 / elapsed_time if elapsed_time > 0 else 0
                    self.start_time = current_time
                
                # 如果有深度数据，则进行融合显示
                if self.depth_image is not None:
                    self.process_fusion()
                elif self.frame_count % 60 == 0:  # 每2秒提醒一次
                    self.get_logger().info('⏳ 等待深度图像数据...')
                    
        except Exception as e:
            self.get_logger().error(f'❌ 彩色图像处理错误: {e}')
            import traceback
            self.get_logger().error(f'详细错误: {traceback.format_exc()}')

    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            with self.data_lock:
                # 深度图像通常是 16-bit 单通道
                self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                
                # 第一次收到深度图像时输出调试信息
                if not hasattr(self, 'depth_received'):
                    self.depth_received = True
                    self.get_logger().info(f'📏 收到第一帧深度图像: {self.depth_image.shape}, dtype: {self.depth_image.dtype}')
                    self.get_logger().info(f'🎯 深度值范围: {np.min(self.depth_image)} - {np.max(self.depth_image)}')
                
        except Exception as e:
            self.get_logger().error(f'❌ 深度图像处理错误: {e}')
            import traceback
            self.get_logger().error(f'详细错误: {traceback.format_exc()}')

    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调函数"""
        self.mouse_x = x
        self.mouse_y = y
        
        # 获取鼠标位置的深度值
        if self.depth_image is not None and self.color_image is not None:
            # 获取彩色图像尺寸
            color_height, color_width = self.color_image.shape[:2]
            depth_height, depth_width = self.depth_image.shape[:2]
            
            # 如果深度图像和彩色图像尺寸不同，需要映射坐标
            if (depth_height, depth_width) != (color_height, color_width):
                # 计算缩放比例
                scale_x = depth_width / color_width
                scale_y = depth_height / color_height
                
                # 映射鼠标坐标到深度图像坐标
                depth_x = int(x * scale_x)
                depth_y = int(y * scale_y)
            else:
                depth_x, depth_y = x, y
            
            # 检查坐标是否在深度图像范围内
            if 0 <= depth_y < depth_height and 0 <= depth_x < depth_width:
                depth_value = self.depth_image[depth_y, depth_x]
                if depth_value > 0:
                    self.mouse_distance = depth_value / 1000.0  # 转换为米
                else:
                    self.mouse_distance = 0.0
            else:
                self.mouse_distance = 0.0
        
        # 处理鼠标点击
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.mouse_distance > 0:
                # 添加点击点到历史记录
                self.click_points.append({
                    'x': x, 'y': y, 
                    'distance': self.mouse_distance,
                    'timestamp': time.time()
                })
                
                unit_distance = self.mouse_distance if self.distance_unit == 'm' else self.mouse_distance * 100
                unit_str = 'm' if self.distance_unit == 'm' else 'cm'
                
                self.get_logger().info(f'📍 点击位置 ({x}, {y}) 的距离: {unit_distance:.1f}{unit_str}')
            else:
                self.get_logger().info(f'📍 点击位置 ({x}, {y}) 无有效深度数据')
        
        # 右键点击切换显示模式
        elif event == cv2.EVENT_RBUTTONDOWN:
            modes = ['color', 'depth', 'overlay']
            current_idx = modes.index(self.display_mode)
            self.display_mode = modes[(current_idx + 1) % len(modes)]
            self.get_logger().info(f'🔄 切换显示模式: {self.display_mode}')

    def process_fusion(self):
        """处理RGB-D融合显示"""
        if self.color_image is None or self.depth_image is None:
            return
            
        if self.gui_available:
            self.display_gui()
        else:
            self.display_console()

    def display_gui(self):
        """在GUI模式下显示融合图像"""
        try:
            color_img = self.color_image.copy()
            depth_img = self.depth_image.copy()
            height, width = color_img.shape[:2]
            
            # 检查深度图像和彩色图像尺寸是否匹配，如果不匹配则调整深度图像尺寸
            if depth_img.shape[:2] != (height, width):
                self.get_logger().debug(f'🔧 调整深度图像尺寸: {depth_img.shape[:2]} -> {(height, width)}')
                depth_img = cv2.resize(depth_img, (width, height), interpolation=cv2.INTER_NEAREST)
            
            # 根据显示模式创建显示图像
            if self.display_mode == 'color':
                display_img = color_img
            elif self.display_mode == 'depth':
                # 深度图像可视化
                depth_normalized = cv2.convertScaleAbs(depth_img, alpha=0.03)
                display_img = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            else:  # overlay
                # 创建深度叠加
                depth_normalized = cv2.convertScaleAbs(depth_img, alpha=0.03)
                depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
                # 将深度图像与彩色图像混合
                display_img = cv2.addWeighted(color_img, 0.7, depth_colored, 0.3, 0)
            
            # 绘制网格（可选）
            if self.show_grid:
                self.draw_grid(display_img, width, height)
            
            # 绘制十字准线
            if self.show_crosshair:
                self.draw_crosshair(display_img, width, height)
            
            # 绘制鼠标位置的距离信息
            self.draw_mouse_info(display_img, width, height)
            
            # 绘制历史点击点
            self.draw_click_points(display_img)
            
            # 绘制统计信息
            self.draw_stats(display_img, width, height)
            
            # 绘制帮助信息
            self.draw_help_overlay(display_img, width, height)
            
            # 显示图像
            cv2.imshow("RealSense RGB-D 融合显示", display_img)
            cv2.setMouseCallback("RealSense RGB-D 融合显示", self.mouse_callback)
            
            # 按键处理
            self.handle_keyboard()
            
        except Exception as e:
            self.get_logger().error(f'❌ GUI显示错误: {e}')
            import traceback
            self.get_logger().error(f'详细错误: {traceback.format_exc()}')

    def draw_grid(self, img, width, height):
        """绘制网格"""
        grid_size = 50
        color = (100, 100, 100)
        
        # 垂直线
        for x in range(0, width, grid_size):
            cv2.line(img, (x, 0), (x, height), color, 1)
        
        # 水平线
        for y in range(0, height, grid_size):
            cv2.line(img, (0, y), (width, y), color, 1)

    def draw_crosshair(self, img, width, height):
        """绘制十字准线"""
        center_x, center_y = width // 2, height // 2
        color = (255, 255, 255)
        thickness = 2
        
        # 十字线
        cv2.line(img, (center_x - 20, center_y), (center_x + 20, center_y), color, thickness)
        cv2.line(img, (center_x, center_y - 20), (center_x, center_y + 20), color, thickness)
        
        # 中心圆
        cv2.circle(img, (center_x, center_y), 5, color, 2)

    def draw_mouse_info(self, img, width, height):
        """绘制鼠标位置信息"""
        if self.mouse_x > 0 and self.mouse_y > 0:
            # 绘制鼠标十字线
            color = (0, 255, 255)  # 黄色
            cv2.line(img, (self.mouse_x - 15, self.mouse_y), 
                    (self.mouse_x + 15, self.mouse_y), color, 2)
            cv2.line(img, (self.mouse_x, self.mouse_y - 15), 
                    (self.mouse_x, self.mouse_y + 15), color, 2)
            
            # 绘制距离信息框
            if self.mouse_distance > 0:
                unit_distance = self.mouse_distance if self.distance_unit == 'm' else self.mouse_distance * 100
                unit_str = 'm' if self.distance_unit == 'm' else 'cm'
                
                distance_text = f"{unit_distance:.1f}{unit_str}"
                coord_text = f"({self.mouse_x}, {self.mouse_y})"
                
                # 计算文本框位置
                text_x = self.mouse_x + 20
                text_y = self.mouse_y - 20
                
                # 确保文本框不超出图像边界
                if text_x + 120 > width:
                    text_x = self.mouse_x - 120
                if text_y - 40 < 0:
                    text_y = self.mouse_y + 40
                
                # 绘制背景框
                cv2.rectangle(img, (text_x - 5, text_y - 35), 
                            (text_x + 110, text_y + 10), (0, 0, 0), -1)
                cv2.rectangle(img, (text_x - 5, text_y - 35), 
                            (text_x + 110, text_y + 10), (255, 255, 255), 1)
                
                # 绘制文本
                cv2.putText(img, distance_text, (text_x, text_y - 15),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(img, coord_text, (text_x, text_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    def draw_click_points(self, img):
        """绘制历史点击点"""
        current_time = time.time()
        
        for i, point in enumerate(self.click_points):
            # 计算点的年龄（秒）
            age = current_time - point['timestamp']
            if age > 10:  # 10秒后移除
                continue
                
            # 根据年龄调整透明度
            alpha = max(0.3, 1.0 - age / 10.0)
            color_intensity = int(255 * alpha)
            
            # 绘制点击点
            color = (0, color_intensity, 0)  # 绿色
            cv2.circle(img, (point['x'], point['y']), 6, color, 2)
            cv2.circle(img, (point['x'], point['y']), 2, color, -1)
            
            # 绘制距离标签
            unit_distance = point['distance'] if self.distance_unit == 'm' else point['distance'] * 100
            unit_str = 'm' if self.distance_unit == 'm' else 'cm'
            text = f"{unit_distance:.1f}{unit_str}"
            
            text_pos = (point['x'] + 10, point['y'] - 10)
            cv2.putText(img, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 
                       0.5, color, 1)

    def draw_stats(self, img, width, height):
        """绘制统计信息"""
        # 计算深度统计
        if self.depth_image is not None:
            valid_depths = self.depth_image[self.depth_image > 0]
            if len(valid_depths) > 0:
                min_dist = np.min(valid_depths) / 1000.0
                max_dist = np.max(valid_depths) / 1000.0
                avg_dist = np.mean(valid_depths) / 1000.0
                
                if self.distance_unit == 'cm':
                    min_dist *= 100
                    max_dist *= 100
                    avg_dist *= 100
                
                unit_str = self.distance_unit
                
                # 信息文本
                info_lines = [
                    f"Frame: {self.frame_count}",
                    f"FPS: {self.fps:.1f}",
                    f"Mode: {self.display_mode.upper()}",
                    f"Range: {min_dist:.1f}-{max_dist:.1f}{unit_str}",
                    f"Avg: {avg_dist:.1f}{unit_str}",
                    f"Points: {len(self.click_points)}"
                ]
                
                # 绘制信息背景
                info_height = len(info_lines) * 20 + 10
                cv2.rectangle(img, (10, 10), (250, info_height), (0, 0, 0), -1)
                cv2.rectangle(img, (10, 10), (250, info_height), (255, 255, 255), 1)
                
                # 绘制信息文本
                for i, line in enumerate(info_lines):
                    y_pos = 30 + i * 20
                    cv2.putText(img, line, (15, y_pos),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def draw_help_overlay(self, img, width, height):
        """绘制帮助信息覆盖层"""
        help_lines = [
            "Keys: q-quit, s-save, h-help, g-grid, c-cross, u-unit",
            "Mouse: Left-measure, Right-mode, Move-preview"
        ]
        
        y_start = height - 40
        for i, line in enumerate(help_lines):
            y_pos = y_start + i * 15
            # 绘制半透明背景
            cv2.rectangle(img, (5, y_pos - 12), (len(line) * 6 + 10, y_pos + 3), 
                         (0, 0, 0), -1)
            cv2.putText(img, line, (8, y_pos), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.4, (200, 200, 200), 1)

    def handle_keyboard(self):
        """处理键盘输入"""
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            self.get_logger().info('👋 用户退出程序')
            rclpy.shutdown()
        elif key == ord('s'):
            self.save_current_frame()
        elif key == ord('h'):
            self.print_help()
        elif key == ord('g'):
            self.show_grid = not self.show_grid
            self.get_logger().info(f'🔲 网格显示: {"开启" if self.show_grid else "关闭"}')
        elif key == ord('c'):
            self.show_crosshair = not self.show_crosshair
            self.get_logger().info(f'✚ 十字准线: {"开启" if self.show_crosshair else "关闭"}')
        elif key == ord('u'):
            self.distance_unit = 'cm' if self.distance_unit == 'm' else 'm'
            self.get_logger().info(f'📏 距离单位: {self.distance_unit}')
        elif key == ord('r'):
            self.click_points.clear()
            self.get_logger().info('🗑️ 清除所有测量点')
        elif key == ord(' '):  # 空格键切换模式
            modes = ['color', 'depth', 'overlay']
            current_idx = modes.index(self.display_mode)
            self.display_mode = modes[(current_idx + 1) % len(modes)]
            self.get_logger().info(f'🔄 切换显示模式: {self.display_mode}')

    def save_current_frame(self):
        """保存当前帧"""
        if self.color_image is not None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            
            # 保存彩色图像
            color_filename = f"rgbd_color_{timestamp}_{self.frame_count:04d}.jpg"
            cv2.imwrite(color_filename, self.color_image)
            
            # 保存深度图像
            if self.depth_image is not None:
                depth_filename = f"rgbd_depth_{timestamp}_{self.frame_count:04d}.png"
                cv2.imwrite(depth_filename, self.depth_image)
            
            # 保存测量点信息
            if self.click_points:
                info_filename = f"rgbd_measurements_{timestamp}_{self.frame_count:04d}.txt"
                with open(info_filename, 'w', encoding='utf-8') as f:
                    f.write(f"RealSense RGB-D 测量数据\n")
                    f.write(f"时间戳: {timestamp}\n")
                    f.write(f"帧号: {self.frame_count}\n")
                    f.write(f"测量点数量: {len(self.click_points)}\n\n")
                    
                    for i, point in enumerate(self.click_points):
                        unit_distance = point['distance'] if self.distance_unit == 'm' else point['distance'] * 100
                        unit_str = self.distance_unit
                        f.write(f"点 {i+1}: ({point['x']}, {point['y']}) = {unit_distance:.1f}{unit_str}\n")
            
            self.get_logger().info(f'💾 已保存: {color_filename} 和相关文件')

    def display_console(self):
        """无GUI模式的终端输出"""
        if self.frame_count % 30 == 0:
            if self.depth_image is not None:
                valid_depths = self.depth_image[self.depth_image > 0]
                if len(valid_depths) > 0:
                    min_dist = np.min(valid_depths) / 1000.0
                    max_dist = np.max(valid_depths) / 1000.0
                    avg_dist = np.mean(valid_depths) / 1000.0
                    
                    self.get_logger().info(f'📊 帧 #{self.frame_count}: FPS {self.fps:.1f}')
                    self.get_logger().info(f'   📏 深度范围: {min_dist:.3f}m - {max_dist:.3f}m')
                    self.get_logger().info(f'   📈 平均深度: {avg_dist:.3f}m')

    def print_help(self):
        """打印帮助信息"""
        help_text = """
🔧 RGB-D 融合显示器控制帮助:

键盘控制:
   q - 退出程序
   s - 保存当前帧和测量数据
   h - 显示此帮助信息
   g - 切换网格显示
   c - 切换十字准线显示
   u - 切换距离单位 (m/cm)
   r - 清除所有测量点
   空格 - 切换显示模式 (彩色/深度/叠加)

鼠标控制:
   移动 - 实时显示距离预览
   左键 - 添加测量点
   右键 - 切换显示模式

显示模式:
   color - 彩色图像 + 距离信息
   depth - 深度图像可视化
   overlay - 彩色+深度叠加显示

功能特点:
   - 实时距离测量
   - 历史测量点记录
   - 多种可视化模式
   - 数据保存功能
        """
        print(help_text)
        self.get_logger().info('📋 帮助信息已显示在终端')

def main(args=None):
    """主函数"""
    print("🎯 启动 RealSense RGB-D 融合显示器...")
    print("💡 功能特点:")
    print("   - 在彩色图像上显示距离信息")
    print("   - 鼠标实时距离预览")
    print("   - 点击测量并保存历史点")
    print("   - 多种显示模式切换")
    print("   - 按 'h' 查看完整帮助")
    print("   - 按 'q' 或 Ctrl+C 退出")
    print()
    
    rclpy.init(args=args)
    
    fusion_display = RGBDepthFusion()
    
    try:
        rclpy.spin(fusion_display)
    except KeyboardInterrupt:
        print("\n🛑 接收到键盘中断信号")
    except Exception as e:
        print(f"❌ 运行时错误: {e}")
    finally:
        if rclpy.ok():
            fusion_display.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()
        print("✅ 程序已正常退出")

if __name__ == '__main__':
    main()