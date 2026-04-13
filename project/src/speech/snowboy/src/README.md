### 1. 构建顺序要求

**必须先构建snoeboy_python模块，再构建调用项目：**

```bash
# 第一步：构建snoeboy_python库
colcon build --packages-select snoeboy_python

# 第二步：构建调用项目（如snowboy_ros）
colcon build --packages-select snowboy_ros
```

### 2. 模型加载
模型在文件夹resources/models/
.pmdl .umdl
hoson.pmdl模型为语音（小浩，小浩）

### 如何创建语音模型
唤醒词训练
snowboy 提供在线自定义的唤醒词，语音训练地址：https://snowboy.hahack.com

输入模型名称
点击 "Recode" 录制 3 段语音，每段 1-2 秒，然后点击 Submit 按钮，等待训练完成。
点击 "Save Model" 保存模型，下载模型文件， 训练的模型文件后缀为 .pmdl。
将下载的模型文件放入 resources/models 目录下，重新运行示例程序

### 注意输出模型为 模型要求参数: 16000Hz, 1声道
### 输入检测音频也应该一致，不一致的音频数据需要输入，频率和声道数进行采样转换

### 3.  运行测试
source install/setup.bash

ros2 run snowboy_ros snowboy_ros hoson.pmdl /home/user/project/src/snowboy/src/resources/MyHoson2.wav 0.5

