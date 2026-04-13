#!/usr/bin/env python3
import os
import sys
import signal
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path

# 设置资源路径
current_file = Path(__file__).resolve()
SNOWBOY_ROOT = 'src/speech/snowboy/src'
RESOURCE_FILE = os.path.join(SNOWBOY_ROOT, "resources/common.res")
MODEL_FILE = os.path.join(SNOWBOY_ROOT, "resources/models")
print("当前文件路径:")
print(current_file)

# 导入核心库
try:
    from snowboy_python.simpledetector import SimpleDetector
    print("✅ 成功导入 SimpleDetector")
except ImportError as e:
    print(f"❌ 无法导入 SimpleDetector: {e}")
    sys.exit(1)

class VoiceWakeupNode(Node):
    def __init__(self, model_path, sensitivity=0.5):
        super().__init__('voice_wakeup_node')
        
        # 创建发布者 - 发布唤醒事件
        self.wakeup_publisher = self.create_publisher(
            String, 
            '/voice_wakeup/detection', 
            10
        )
        model_file = os.path.join(MODEL_FILE, model_path)
        res_file =  RESOURCE_FILE
        self.get_logger().info(f"模型路径: {model_file}")
        self.get_logger().info(f"模型路径: {res_file}")
        # 创建检测器
        try:
            
            self.detector = SimpleDetector(model_file, res_file, sensitivity=sensitivity)
            # 设置ROS节点引用，用于发布消息
            # self.detector.set_ros_node = self.set_ros_node
            # self.detector.publish_wakeup_event = self.publish_wakeup_event
        except Exception as e:
            self.get_logger().error(f"创建检测器失败: {e}")
            raise
        
        self.get_logger().info("语音唤醒节点已启动")
        self.get_logger().info(f"唤醒消息将发布到: /voice_wakeup/detection")

    def set_ros_node(self, ros_node):
        """设置ROS2节点引用"""
        self.ros_node = ros_node
        
    def publish_wakeup_event(self, detection_id, timestamp):
        """发布唤醒事件"""
        msg = String()
        msg.data = f"wakeup:{detection_id}:{timestamp:.2f}"
        self.wakeup_publisher.publish(msg)
        self.get_logger().info(f"📢 发布唤醒事件: ID={detection_id}, 时间={timestamp:.2f}s")

def main():
    # 初始化ROS2
    rclpy.init()
    
    if len(sys.argv) < 2:
        print("使用方法:")
        print("  ros2 run snowboy_ros snowboy_ros 模型路径 [音频文件路径] [敏感度]")
        print("")
        print("示例:")
        print("  # 实时监听模式")
        print("  ros2 run snowboy_ros snowboy_ros src/snowboy/src/resources/models/hoson.pmdl")
        print("")
        print("  # 测试音频文件模式")
        print("  ros2 run snowboy_ros snowboy_ros src/snowboy/src/resources/models/hoson.pmdl /path/to/audio.wav")
        print("")
        print("  # 指定敏感度测试")
        print("  ros2 run snowboy_ros snowboy_ros src/snowboy/src/resources/models/hoson.pmdl /path/to/audio.wav 0.5")
        return
        
    model_path = sys.argv[1]
    
    # 获取敏感度参数（默认0.5）
    sensitivity = 0.5
    if len(sys.argv) >= 4:
        try:
            sensitivity = float(sys.argv[3])
        except ValueError:
            print("错误: 敏感度必须是数字")
            return
    
    # 创建ROS2节点
    try:
        node = VoiceWakeupNode(model_path, sensitivity=sensitivity)
    except Exception as e:
        print(f"❌ 创建节点失败: {e}")
        rclpy.shutdown()
        return
    
    # 设置信号处理
    def signal_handler(sig, frame):
        print("\n收到停止信号...")
        node.detector.stop()
        rclpy.shutdown()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    # 检查是否有音频文件参数
    if len(sys.argv) >= 3:
        audio_file_path = sys.argv[2]
        
        print("=" * 50)
        print("🎵 音频文件测试模式")
        print(f"🎯 使用敏感度: {sensitivity}")
        print("📢 唤醒消息将发布到: /voice_wakeup/detection")
        print("=" * 50)
        
        # 测试音频文件
        node.detector.test_audio_file(audio_file_path)
        
        # 测试完成后关闭节点
        rclpy.shutdown()
            
    else:
        # 实时监听模式
        print("=" * 50)
        print("🎧 实时监听模式")
        print(f"🎯 使用敏感度: {sensitivity}")
        print("📢 唤醒消息将发布到: /voice_wakeup/detection")
        print("按 Ctrl+C 停止监听")
        print("=" * 50)
        
        # 开始实时检测
        node.detector.start_detection()
        
        # 在检测运行时保持ROS2节点活跃
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()