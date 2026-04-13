#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include <cstdio>
#include <csignal>

// 全局变量用于存储原始终端设置
static struct termios original_settings;
static bool terminal_settings_saved = false;

// 信号处理函数，确保终端设置被正确恢复
void signal_handler(int signal) {
    if (terminal_settings_saved) {
        // 恢复终端设置
        tcsetattr(0, TCSANOW, &original_settings);
        // 清空输入缓冲区
        tcflush(0, TCIFLUSH);
    }
    // 退出程序
    exit(signal);
}

// 获取终端属性以支持非阻塞键盘输入
class KeyboardReader
{
public:
    KeyboardReader()
    {
        tcgetattr(0, &initial_settings_);
        new_settings_ = initial_settings_;
        new_settings_.c_lflag &= ~ICANON;
        new_settings_.c_lflag &= ~ECHO;
        new_settings_.c_cc[VMIN] = 1;
        new_settings_.c_cc[VTIME] = 0;
        tcsetattr(0, TCSANOW, &new_settings_);
        fcntl(0, F_SETFL, O_NONBLOCK);
    }

    ~KeyboardReader()
    {
        // 恢复终端设置
        tcsetattr(0, TCSANOW, &initial_settings_);
        // 清空输入缓冲区
        tcflush(0, TCIFLUSH);
    }

    int readOne()
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

private:
    struct termios initial_settings_;
    struct termios new_settings_;
};

int main(int argc, char **argv)
{
    // 注册信号处理函数，处理Ctrl+C等信号
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // 保存原始终端设置
    tcgetattr(0, &original_settings);
    terminal_settings_saved = true;
    
    std::cout << "Simple keyboard test started. Press 'q' to quit." << std::endl;
    
    // 创建键盘读取器
    KeyboardReader key_reader;
    
    bool running = true;
    while (running)
    {
        // 读取键盘输入
        int key = key_reader.readOne();
        if (key != -1) {
            std::cout << "Detected key: " << (char)key << " (" << key << ")" << std::endl;
            if (key == 'q' || key == 'Q') {
                running = false;
            }
        }
        // 短暂休眠以避免过度占用CPU
        usleep(50000);  // 50ms
    }
    
    std::cout << "Keyboard test finished." << std::endl;
    return 0;
}