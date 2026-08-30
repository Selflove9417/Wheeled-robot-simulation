#include "bbot_balance_controller/keyboard_reader.h"

#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <cerrno>

KeyboardReader::KeyboardReader()
    : terminal_modified_(false)
{
    // 获取当前终端属性
    if (tcgetattr(STDIN_FILENO, &original_tios_) < 0)
    {
        // 无法获取终端属性（可能没有连接终端），跳过设置
        return;
    }

    struct termios raw = original_tios_;

    // 关闭行缓冲和回显
    // ICANON: 规范模式（行缓冲）
    // ECHO:   输入回显
    raw.c_lflag &= ~(ICANON | ECHO);

    // 设置最小读取字符数和超时
    // VMIN = 0, VTIME = 0：read 立即返回，不等待
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0)
    {
        return;
    }

    terminal_modified_ = true;
}

KeyboardReader::~KeyboardReader()
{
    restore_terminal();
}

void KeyboardReader::restore_terminal()
{
    if (terminal_modified_)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_tios_);
        terminal_modified_ = false;
    }
}

char KeyboardReader::read_char_nonblock()
{
    char c = 0;
    int n = read(STDIN_FILENO, &c, 1);
    if (n <= 0)
        return 0;
    return c;
}

char KeyboardReader::read_key()
{
    if (!terminal_modified_)
        return 0;

    std::lock_guard<std::mutex> lock(mutex_);

    char c = read_char_nonblock();
    if (c == 0)
        return 0;

    // 如果是 ESC 序列开头（如方向键），在当前帧直接忽略
    // 完整的方向键处理由 read_sequence() 负责
    // 这里只处理单字节普通按键
    if (c == 27)  // ESC
    {
        // 尝试读取后续两个字节
        char seq[2] = {0, 0};
        read(STDIN_FILENO, &seq[0], 1);
        read(STDIN_FILENO, &seq[1], 1);

        // 如果后续读取到了 [ ，继续读方向键标识
        // 这里我们直接丢弃方向键序列，只返回单字符按键
        return 0;
    }

    return c;
}

std::string KeyboardReader::read_sequence()
{
    if (!terminal_modified_)
        return "";

    char c = read_char_nonblock();
    if (c == 0)
        return "";

    if (c == 27)  // ESC
    {
        char c2 = read_char_nonblock();
        if (c2 != '[')
        {
            // 不是方向键序列
            return "";
        }

        char c3 = read_char_nonblock();
        switch (c3)
        {
            case 'A': return "UP";
            case 'B': return "DOWN";
            case 'C': return "RIGHT";
            case 'D': return "LEFT";
            default:  return "";
        }
    }

    // 单字符按键
    return std::string(1, c);
}