#include "speech_commands.hpp"
#include <algorithm>

namespace cmd_dispatcher
{
    // 语音命令映射表
    const std::map<SpeechCmdType, std::vector<std::string>> speech_commands = {
    {SpeechCmdType::UNKNOWN_SPEECH, {"未知语义信息"}},
    {SpeechCmdType::WELCOME_SPEECH, {"欢迎词"}},
    {SpeechCmdType::TURN_ON_SPEECH, {"开启播报","开播报", "打开播报", "开始播报"}},  
    {SpeechCmdType::TURN_OFF_SPEECH, {"关闭播报","关播报", "关闭语音", "禁止播报", "关掉播报"}},
    {SpeechCmdType::VOLUME_UP, {"调大音量", "调大声音","增大音量", "增大声音", "提高音量", "声音大一点", "音量加大", "声音加大"}},
    {SpeechCmdType::VOLUME_DOWN, {"调小音量","调小声音", "减小音量","减小声音", "降低音量", "声音小一点", "音量减小", "声音减小"}},
    {SpeechCmdType::VOLUME_MAX, {"最大音量", "最大声音", "音量最大", "声音最大",  "调到最大音量","调到最大声音", "把音量调到最大", "把声音调到最大"}},
    {SpeechCmdType::VOLUME_MIN, {"最小音量", "最小声音", "音量最小", "声音最小", "调到最小音量","调到最小声音", "把音量调到最小", "把声音调到最小"}},
    {SpeechCmdType::VOLUME_MEDIUM, {"中等音量", "中等声音", "音量中等", "声音中等",  "调到中等音量", "调到中等声音", "把音量调到中等", "把声音调到中等"}},
    {SpeechCmdType::STOP_MOVE, {"停下","停止移动", "停止前进", "不要走了", "停下来", "停", "stop"}},
    {SpeechCmdType::MOVE_FORWARD, {"向前走", "朝前走", "往前走", "前进"}},
    {SpeechCmdType::MOVE_BACKWARD, {"向后走", "朝后走", "往后走", "向后退","后退"}},
    {SpeechCmdType::MOVE_TO_BOX, {"走到箱子位置", "走到箱子的位置", "走到前面箱子那里", "走到前面箱子那里去", "走到箱子前面去", "走到前面的箱子位置",  "走到箱子那个位置", "走到前面的箱子那个位置", "去箱子那里", "到箱子的位置"}},
    {SpeechCmdType::MOVE_TO_DESK, {"走到桌子位置", "走到桌子的位置", "走到前面桌子那里", "走到前面桌子那里去", "走到桌子前面去", "走到前面的桌子位置", "走到桌子那个位置", "走到前面的桌子那个位置", "去桌子那里", "到桌子的位置"}},
    {SpeechCmdType::MOVE_TO_CHAIR, {"走到椅子位置", "走到椅子的位置", "走到前面椅子那里", "走到前面椅子那里去", "走到椅子前面去", "走到前面的椅子位置", "走到椅子那个位置", "走到前面的椅子那个位置", "去椅子那里", "到椅子的位置"}},
    {SpeechCmdType::DETECT_LAPTOP, {"检测笔记本电脑","找到笔记本电脑", "查找笔记本电脑", "寻找笔记本电脑"}},
    {SpeechCmdType::SING_A_SONG, {"唱歌", "唱首歌", "唱一首歌","来首歌"}},
    {SpeechCmdType::DANCE, {"跳舞", "来段舞蹈", "跳个舞"}},
    };

int levenshtein(const std::string &s1, const std::string &s2)
{
  const size_t m = s1.size(), n = s2.size();
  std::vector<int> prev(n + 1), curr(n + 1);
  for (size_t j = 0; j <= n; ++j) prev[j] = static_cast<int>(j);
  for (size_t i = 1; i <= m; ++i) {
    curr[0] = static_cast<int>(i);
    for (size_t j = 1; j <= n; ++j) {
      int cost = (s1[i - 1] == s2[j - 1]) ? 0 : 1;
      curr[j] = std::min({curr[j - 1] + 1, prev[j] + 1, prev[j - 1] + cost});
    }
    std::swap(prev, curr);
  }
  return prev[n];
}

double similarity(const std::string &s1, const std::string &s2)
{
  if (s1.empty() && s2.empty()) return 1.0;
  int dist = levenshtein(s1, s2);
  int maxlen = static_cast<int>(std::max(s1.size(), s2.size()));
  return 1.0 - static_cast<double>(dist) / static_cast<double>(maxlen);
}

// 获取语音命令的中文语义
std::string get_speech_meaning(SpeechCmdType command_type) 
{
  auto it = speech_commands.find(command_type);
  if (it != speech_commands.end() && !it->second.empty()) 
  {
    return it->second.front();  // 返回第一个字符串作为语义信息
  }
  return "未知命令";
}

// 根据文本获取解析的命令类型
SpeechCmdType get_speech_command(const std::string& voice_text)
{
    for (const auto& it : speech_commands) 
    {
        // 遍历每个命令类型对应的多个文本
        for (const auto& command_text : it.second) 
        {
            if (similarity(voice_text, command_text) >= 0.80)   // 80 % 阈值
                return it.first;
        }
    }
    return SpeechCmdType::UNKNOWN_SPEECH; // 默认返回未知命令
}

// 获取语音命令的回复内容
std::string get_speech_reply(SpeechCmdType command_type, const std::string& voice_text) 
{
    std::string strMsg;
    switch (command_type)
    {   
    case SpeechCmdType::WELCOME_SPEECH: 
        if (voice_text.find("你好") != std::string::npos) 
            strMsg = "你好，我在";
        else
            strMsg = "我在"; // 欢迎词
        break;
    case SpeechCmdType::TURN_ON_SPEECH: 
        strMsg = "好的，我将开始播报";
        break;
    case SpeechCmdType::TURN_OFF_SPEECH: 
        if (voice_text.find("播报") != std::string::npos) 
            strMsg = "好的，我去休息了";
        else
            strMsg = "我去休息了"; 
        break;
    case SpeechCmdType::VOLUME_UP:
        strMsg = "好的，我将调大音量";
        break;
    case SpeechCmdType::VOLUME_DOWN:
        strMsg = "好的，我将调小音量";
        break;
    case SpeechCmdType::VOLUME_MAX:
        strMsg = "好的，我将音量调到最大";
        break;
    case SpeechCmdType::VOLUME_MIN:
        strMsg = "好的，我将音量调到最小";
        break;
    case SpeechCmdType::VOLUME_MEDIUM:
        strMsg = "好的，我将音量调到中等";
        break;
    case SpeechCmdType::STOP_MOVE: 
        strMsg = "好的，我将停止移动";
        break;
    case SpeechCmdType::MOVE_FORWARD: 
        strMsg = "好的，我将往前走";
        break;
    case SpeechCmdType::MOVE_BACKWARD: 
        strMsg = "好的，我将往后走";
        break;
    case SpeechCmdType::MOVE_TO_BOX: 
        strMsg = "好的，我将走到箱子位置";
        break;
    case SpeechCmdType::MOVE_TO_DESK: 
        strMsg = "好的，我将走到桌子位置";
        break;
    case SpeechCmdType::MOVE_TO_CHAIR: 
        strMsg = "好的，我将走到椅子位置";
        break;
    case SpeechCmdType::DETECT_LAPTOP: 
        strMsg = "好的，我将搜寻笔记本电脑";
        break;
    case SpeechCmdType::SING_A_SONG: 
        strMsg = "五星红旗迎风飘扬，胜利歌声多么响亮！";
        break;
    case SpeechCmdType::DANCE: 
        strMsg = "好的，我将跳舞";
        break;
    default: 
        strMsg = "抱歉，您的表达我暂时不理解。";
        break;
    }
    return strMsg;
}

//获取动作任务ID
int get_act_task_id(SpeechCmdType command_type)
{
    switch (command_type)
    {
    case SpeechCmdType::VOLUME_UP:
        return 10;
    case SpeechCmdType::VOLUME_DOWN:
        return 11;
    case SpeechCmdType::VOLUME_MAX:
        return 12;
    case SpeechCmdType::VOLUME_MIN:
        return 13;
    case SpeechCmdType::VOLUME_MEDIUM:
        return 14;
    case SpeechCmdType::STOP_MOVE:
        return 15;
    case SpeechCmdType::MOVE_FORWARD:
        return 16;
    case SpeechCmdType::MOVE_BACKWARD:
        return 17;
    case SpeechCmdType::MOVE_TO_BOX:
        return 18;
    case SpeechCmdType::MOVE_TO_DESK:
        return 19;
    case SpeechCmdType::MOVE_TO_CHAIR:
        return 20;  // 为椅子位置分配任务ID 5
    case SpeechCmdType::DETECT_LAPTOP:
        return 21;
    case SpeechCmdType::SING_A_SONG:
        return 22;
    case SpeechCmdType::DANCE:
        return 23;
    default:
        return -1;
    }
}

} // namespace cmd_dispatcher
