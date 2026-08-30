#ifndef BBOT_BALANCE_CONTROLLER_KEYBOARD_READER_H
#define BBOT_BALANCE_CONTROLLER_KEYBOARD_READER_H

#include <termios.h>
#include <string>
#include <vector>
#include <mutex>

class KeyboardReader
{
public:
    KeyboardReader();
    ~KeyboardReader();

    // 非阻塞读取，返回最新按键字符；无按键时返回 0
    char read_key();

    // 读上行键序列（如方向键 ESC-sequences）；无完整序列时返回 0，否则返回对应的逻辑键名
    std::string read_sequence();

private:
    void restore_terminal();
    char read_char_nonblock();

    struct termios original_tios_;
    bool terminal_modified_;
    std::mutex mutex_;
};

#endif