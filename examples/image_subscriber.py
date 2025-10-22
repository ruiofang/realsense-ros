#!/usr/bin/env python3
"""
RealSense 彩色图像订阅者示例
订阅并显示 RealSense 相机的彩色图像
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    """彩色图像订阅者类"""
    
    def __init__(self):
        super().__init__('realsense_image_subscriber')
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        # CV Bridge 用于图像格式转换
        self.bridge = CvBridge()
        
        # 计数器
        self.frame_count = 0
        
        self.get_logger().info('🚀 RealSense 彩色图像订阅者已启动')
        self.get_logger().info('📷 订阅话题: /camera/camera/color/image_raw')
        self.get_logger().info('🔄 按 Ctrl+C 退出程序')

    def image_callback(self, msg):
        """图像回调函数"""
        try:
            # 将 ROS 图像消息转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 获取图像信息
            height, width, channels = cv_image.shape
            self.frame_count += 1
            
            # 在图像上添加信息文本
            info_text = f"Frame: {self.frame_count} | Size: {width}x{height}"
            cv2.putText(cv_image, info_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示图像
            cv2.imshow("RealSense 彩色图像", cv_image)
            
            # 按键处理
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('👋 用户退出程序')
                rclpy.shutdown()
            elif key == ord('s'):
                # 保存当前帧
                filename = f"realsense_frame_{self.frame_count:04d}.jpg"
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'💾 已保存图像: {filename}')
            
            # 每100帧打印一次信息
            if self.frame_count % 100 == 0:
                self.get_logger().info(f'📊 已处理 {self.frame_count} 帧图像')
            
        except Exception as e:
            self.get_logger().error(f'❌ 图像处理错误: {e}')

def main(args=None):
    """主函数"""
    print("🎯 启动 RealSense 彩色图像订阅者...")
    print("💡 提示:")
    print("   - 按 'q' 键退出程序")
    print("   - 按 's' 键保存当前帧")
    print("   - 按 Ctrl+C 也可退出")
    print()
    
    rclpy.init(args=args)
    
    image_subscriber = ImageSubscriber()
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        print("\n🛑 接收到键盘中断信号")
    except Exception as e:
        print(f"❌ 运行时错误: {e}")
    finally:
        # 清理资源
        if rclpy.ok():
            image_subscriber.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()
        print("✅ 程序已正常退出")

if __name__ == '__main__':
    main()