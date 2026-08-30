#include "bbot_motor_driver/can_interface.hpp"

#include <unistd.h>
#include <cstring>

#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can/raw.h>
#include <net/if.h>

CanInterface::CanInterface()
: socket_fd_(-1)
{
}

CanInterface::~CanInterface()
{
    if (socket_fd_ >= 0)
    {
        close(socket_fd_);
    }
}

bool CanInterface::open(const std::string &ifname)
{
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (socket_fd_ < 0)
    {
        return false;
    }

    struct ifreq ifr;

    strcpy(ifr.ifr_name, ifname.c_str());

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
    {
        return false;
    }

    struct sockaddr_can addr;

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_,
             (struct sockaddr *)&addr,
             sizeof(addr)) < 0)
    {
        return false;
    }

    return true;
}

bool CanInterface::sendFrame(
    uint32_t can_id,
    const uint8_t *data,
    uint8_t dlc)
{
    struct can_frame frame;

    frame.can_id = can_id;
    frame.can_dlc = dlc;

    memcpy(frame.data, data, dlc);

    return write(socket_fd_,
                 &frame,
                 sizeof(frame))
           == sizeof(frame);
}

bool CanInterface::receiveFrame(
    struct can_frame &frame)
{
    return read(socket_fd_,
                &frame,
                sizeof(frame))
           == sizeof(frame);
}
