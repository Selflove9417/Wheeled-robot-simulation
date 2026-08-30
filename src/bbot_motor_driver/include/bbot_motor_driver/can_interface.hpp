#pragma once

#include <string>
#include <linux/can.h>

class CanInterface
{
public:

    CanInterface();

    ~CanInterface();

    bool open(const std::string &ifname);

    bool sendFrame(
        uint32_t can_id,
        const uint8_t *data,
        uint8_t dlc);

    bool receiveFrame(
        struct can_frame &frame);

private:

    int socket_fd_;
};