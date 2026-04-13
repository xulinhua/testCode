"""
测试音量控制功能
"""
import time
from audio_io_manager import AudioIOManager

def test_volume_control():
    """测试音量控制功能"""
    print("=== 音量控制测试 ===")
    
    # 创建音频管理器实例
    print("1. 创建音频管理器实例...")
    audio_manager = AudioIOManager(input_device_index=2, output_device_index=3)
    print("   设备支持的默认采样率:", audio_manager.default_rate)
    
    try:
        # 测试音量控制
        print("\n2. 测试音量控制功能...")
        print("   初始音量增益:", audio_manager.volume_gain)
        
        # 设置不同的音量增益值
        test_gains = [0.5, 1.0, 1.5, 2.0]
        for gain in test_gains:
            audio_manager.set_volume_gain(gain)
            print(f"   设置音量增益为 {gain}: {audio_manager.volume_gain}")
            
        # 测试边界值
        print("\n3. 测试边界值...")
        audio_manager.set_volume_gain(-0.5)  # 应该被限制为0.0
        print(f"   设置音量增益为 -0.5: {audio_manager.volume_gain}")
        
        audio_manager.set_volume_gain(3.0)  # 应该被限制为2.0
        print(f"   设置音量增益为 3.0: {audio_manager.volume_gain}")
        
        # 恢复默认值
        audio_manager.set_volume_gain(1.0)
        print(f"   恢复默认音量增益: {audio_manager.volume_gain}")
        
        print("\n=== 音量控制测试完成 ===")
        
    except Exception as e:
        print(f"发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        audio_manager.close()

if __name__ == "__main__":
    test_volume_control()