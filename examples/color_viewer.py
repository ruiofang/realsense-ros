#!/usr/bin/env python3
"""
RealSense 彩色图像订阅器 - 简化版本
订阅并显示 RealSense 相机的彩色图像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time

class ColorImageSubscriber(Node):
    """彩色图像订阅器类"""
    
    def __init__(self):
        super().__init__('realsense_color_subscriber')
        
        # 检查 GUI 支持
        self.gui_available = self.check_gui_support()
        
        # 订阅彩色图像
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0
        
        self.get_logger().info('🚀 RealSense 彩色图像订阅器已启动')
        self.get_logger().info('📸 订阅话题: /camera/camera/color/image_raw')
        
        if self.gui_available:
            self.get_logger().info('🖼️  GUI可用: 将显示彩色图像窗口')
        else:
            self.get_logger().info('📊 无GUI模式: 将输出图像统计信息到终端')
            
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

    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 转换图像格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.frame_count += 1
            
            # 计算 FPS
            current_time = time.time()
            if self.frame_count % 30 == 0:  # 每30帧计算一次FPS
                elapsed_time = current_time - self.start_time
                self.fps = 30 / elapsed_time if elapsed_time > 0 else 0
                self.start_time = current_time
            
            if self.gui_available:
                # GUI 模式 - 显示图像
                self.display_gui(cv_image)
            else:
                # 无GUI模式 - 输出到终端
                self.display_console(cv_image)
            
        except Exception as e:
            import traceback
            self.get_logger().error(f'❌ 图像处理错误: {e}')
            self.get_logger().error(f'详细错误: {traceback.format_exc()}')
    
    def display_gui(self, image):
        """在GUI模式下显示图像"""
        try:
            # 获取图像信息
            height, width, channels = image.shape
            
            # 创建显示图像的副本
            display_image = image.copy()
            
            # 在图像上添加信息
            info_text = f"Frame: {self.frame_count} | Size: {width}x{height} | FPS: {self.fps:.1f}"
            cv2.putText(display_image, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 添加时间戳
            timestamp = time.strftime("%H:%M:%S")
            cv2.putText(display_image, timestamp, (width - 120, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # 在中心绘制十字线
            center_x, center_y = width // 2, height // 2
            cv2.line(display_image, (center_x - 20, center_y), 
                    (center_x + 20, center_y), (0, 255, 255), 2)
            cv2.line(display_image, (center_x, center_y - 20), 
                    (center_x, center_y + 20), (0, 255, 255), 2)
            
            # 显示图像
            cv2.imshow("RealSense 彩色图像", display_image)
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('👋 用户退出程序')
                rclpy.shutdown()
            elif key == ord('s'):
                # 保存图像
                filename = f"color_image_{self.frame_count:06d}_{time.strftime('%H%M%S')}.jpg"
                cv2.imwrite(filename, image)
                self.get_logger().info(f'💾 已保存图像: {filename}')
            elif key == ord('h'):
                # 显示帮助信息
                self.print_help()
            elif key == ord('f'):
                # 全屏切换
                self.toggle_fullscreen()
                
        except Exception as e:
            self.get_logger().error(f'❌ GUI显示错误: {e}')
            # 如果GUI出错，切换到无GUI模式
            self.gui_available = False
            self.get_logger().info('🔄 切换到无GUI模式')
            
    def display_console(self, image):
        """在无GUI模式下输出到终端"""
        height, width, channels = image.shape
        
        # 每30帧输出一次详细信息
        if self.frame_count % 30 == 0:
            # 计算图像统计信息
            mean_color = np.mean(image, axis=(0, 1))
            brightness = np.mean(mean_color)
            
            self.get_logger().info(f'📊 帧 #{self.frame_count}:')
            self.get_logger().info(f'   📐 尺寸: {width}x{height}x{channels}')
            self.get_logger().info(f'   📈 FPS: {self.fps:.1f}')
            self.get_logger().info(f'   💡 亮度: {brightness:.1f}')
            self.get_logger().info(f'   🎨 平均颜色 (BGR): ({mean_color[0]:.0f}, {mean_color[1]:.0f}, {mean_color[2]:.0f})')
            
        # 每10帧输出简要信息
        elif self.frame_count % 10 == 0:
            self.get_logger().info(f'帧 #{self.frame_count}: {width}x{height}, FPS: {self.fps:.1f}')
    
    def toggle_fullscreen(self):
        """切换全屏模式"""
        try:
            prop = cv2.getWindowProperty("RealSense 彩色图像", cv2.WND_PROP_FULLSCREEN)
            if prop == cv2.WINDOW_FULLSCREEN:
                cv2.setWindowProperty("RealSense 彩色图像", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                self.get_logger().info('🪟 切换到窗口模式')
            else:
                cv2.setWindowProperty("RealSense 彩色图像", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                self.get_logger().info('🖥️  切换到全屏模式')
        except Exception as e:
            self.get_logger().error(f'全屏切换失败: {e}')

    def print_help(self):
        """打印帮助信息"""
        help_text = """
🔧 彩色图像订阅器控制帮助:
   q - 退出程序
   s - 保存当前图像
   f - 切换全屏/窗口模式
   h - 显示此帮助信息
   
📊 显示信息说明:
   - 绿色文字: 帧号、尺寸、FPS
   - 白色文字: 时间戳
   - 黄色十字: 图像中心
        """
        print(help_text)
        self.get_logger().info('📋 帮助信息已显示在终端')

def main(args=None):
    """主函数"""
    print("🎯 启动 RealSense 彩色图像订阅器...")
    print("💡 提示:")
    print("   - 按 's' 键保存当前图像")
    print("   - 按 'f' 键切换全屏模式")
    print("   - 按 'h' 键查看完整帮助")
    print("   - 按 'q' 键或 Ctrl+C 退出")
    print()
    
    rclpy.init(args=args)
    
    color_subscriber = ColorImageSubscriber()
    
    try:
        rclpy.spin(color_subscriber)
    except KeyboardInterrupt:
        print("\n🛑 接收到键盘中断信号")
    except Exception as e:
        print(f"❌ 运行时错误: {e}")
    finally:
        # 清理资源
        if rclpy.ok():
            color_subscriber.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()
        print("✅ 程序已正常退出")

if __name__ == '__main__':
    main()