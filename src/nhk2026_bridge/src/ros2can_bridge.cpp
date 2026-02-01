#include "ros2can_bridge.hpp"

CanBridge::CanBridge(const char* Ifname)
: ifname(Ifname)
{
    this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (this->sock < 0)
    {
        throw std::runtime_error("failed to open socket");
    }

    int enable_canfd = 1;
    if (setsockopt(this->sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
    &enable_canfd, sizeof(enable_canfd)) < 0)
    {
        close(this->sock);
        throw std::runtime_error("failed to socketopt canfd");
    }

    ifreq ifr{};
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(this->sock, SIOCGIFINDEX, &ifr) < 0)
    {
        close(this->sock);
        throw std::runtime_error("failed to use ioctl");
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(this->sock, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        close(this->sock);
        throw std::runtime_error("failed to bind");
    }
}

CanBridge::~CanBridge()
{
    close(this->sock);
}

void CanBridge::send_float(int canid, std::vector<float> txdata_f)
{
    int byte_length = (int)txdata_f.size() * 4;
    if (byte_length > 64)
    {
        throw std::runtime_error("Data Length is too long");
    }

    canfd_frame frame{};
    frame.can_id = canid;
    frame.len = byte_length;
    frame.flags |= CANFD_BRS;
    for (int i = 0; i < (int)txdata_f.size(); i++)
    {
        union Data data;
        data.data_f32 = txdata_f[i];
        frame.data[i*4    ] = (uint8_t)((data.data_ui32 >> 24) & 0xff);
        frame.data[i*4 + 1] = (uint8_t)((data.data_ui32 >> 16) & 0xff);
        frame.data[i*4 + 2] = (uint8_t)((data.data_ui32 >>  8) & 0xff);
        frame.data[i*4 + 3] = (uint8_t)((data.data_ui32      ) & 0xff);
    }
    int nbytes = write(this->sock, &frame, sizeof(frame));
    if (nbytes < 0)
    {
        throw std::runtime_error("failed to write");
    }
}