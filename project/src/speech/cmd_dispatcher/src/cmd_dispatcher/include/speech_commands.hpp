#pragma once
#include <string>
#include <map>
#include <vector>

namespace cmd_dispatcher
{

// 语音命令枚举类
enum class SpeechCmdType {
  UNKNOWN_SPEECH = -1,  //未知语义命令
  WELCOME_SPEECH = 0,   // 欢迎词（自动开启播报）
  TURN_ON_SPEECH,       // 开启播报
  TURN_OFF_SPEECH,      // 关闭播报
  VOLUME_UP,            // 调大音量
  VOLUME_DOWN,          // 调小音量
  VOLUME_MAX,           // 最大音量
  VOLUME_MIN,           // 最小音量
  VOLUME_MEDIUM,        // 中等音量
  STOP_MOVE,            // 停止移动
  MOVE_FORWARD,         // 向前走
  MOVE_BACKWARD,        // 向后走
  MOVE_TO_BOX,          // 走到箱子位置
  MOVE_TO_DESK,         // 走到桌子位置
  MOVE_TO_CHAIR,        // 走到椅子位置
  DETECT_LAPTOP,        // 检测笔记本电脑
  SING_A_SONG,          // 唱一首歌
  DANCE,                // 跳舞
};

// 声明外部映射表和接口函数
extern const std::map<SpeechCmdType, std::vector<std::string>> speech_commands;

// 获取语音命令的中文语义
std::string get_speech_meaning(SpeechCmdType command_type);

// 根据文本获取解析的命令类型
SpeechCmdType get_speech_command(const std::string& voice_text);

// 获取语音命令的回复内容
std::string get_speech_reply(SpeechCmdType command_type, const std::string& voice_text);

//获取动作任务ID
int get_act_task_id(SpeechCmdType command_type);

} // namespace cmd_dispatcher