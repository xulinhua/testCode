#include "marker_detect_ros/keyboard_handler.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>

namespace marker_detect_ros {

KeyboardHandler::KeyboardHandler()
{
    tcgetattr(0, &initial_settings_);
    new_settings_ = initial_settings_;
    new_settings_.c_lflag &= ~ICANON;  // 关闭规范模式
    new_settings_.c_lflag &= ~ECHO;    // 关闭回显
    new_settings_.c_cc[VMIN] = 0;      // 非阻塞模式
    new_settings_.c_cc[VTIME] = 0;     // 无超时
    tcsetattr(0, TCSANOW, &new_settings_);
    fcntl(0, F_SETFL, O_NONBLOCK);
}

KeyboardHandler::~KeyboardHandler()
{
    // 恢复终端设置
    tcsetattr(0, TCSANOW, &initial_settings_);
    // 清空输入缓冲区
    tcflush(0, TCIFLUSH);
}

int KeyboardHandler::readOne()
{
    fd_set set;
    struct timeval timeout;
    FD_ZERO(&set);
    FD_SET(0, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;  // 10ms超时
    
    int res = select(1, &set, NULL, NULL, &timeout);
    if (res > 0)
    {
        char c;
        // 尝试读取字符
        ssize_t bytes_read = read(0, &c, 1);
        if (bytes_read > 0)
            return c;
    }
    else if (res < 0)
    {
        // select出错
        perror("select");
    }
    // 没有数据可读或超时
    return -1;
}


}  // namespace marker_detect_ros
