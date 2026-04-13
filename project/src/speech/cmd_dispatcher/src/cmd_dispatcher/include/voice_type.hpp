#pragma once

namespace cmd_dispatcher
{

// 命令类型枚举定义
enum class Voice_Type : uint8_t {
  UNKNOWN = 0,
  WAKEUP_ENABLE = 1,
  WAKEUP_DISABLE = 2,
  NORMAL_COMMAND = 3
};

} // namespace cmd_dispatcher