### 支持的手势识别

- **Wave** - 挥手（所有手指张开 + 水平移动）
- **Stop** - 停止手势（所有手指张开 + 手掌朝前）
- **Handshake** - 握手手势（所有手指张开 + 手掌侧向）
- **Heart_single** - 单手爱心（拇指和食指靠近）
- **ILY** - 我爱你手势（拇指、食指、小指伸直）
- **Insult** - 竖中指（中指伸直 + 手掌侧向）
- **one** - 数字1（食指伸直）
- **fist** - 拳头（所有手指弯曲）
- **peace** - 胜利手势（食指、中指伸直）
- **call** - 打电话手势（拇指、小指伸直）
- **ok** - OK手势（食指成环）
- **three** - 数字3（拇指、食指、中指伸直）
- **four** - 数字4（除拇指外所有手指伸直）
- **like** - 点赞（拇指向上）
- **dislike** - 踩（拇指向下）

# 安装mediapipe
pip install mediapipe

# 重新构建
colcon build --packages-select gesture_rec

# 重新运行
source install/setup.bash
ros2 launch gesture_rec gesture_rec.py camera_type:=Gemini
