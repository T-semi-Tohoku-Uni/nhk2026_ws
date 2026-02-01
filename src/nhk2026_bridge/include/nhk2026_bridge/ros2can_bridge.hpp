#pragma once

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <vector>
#include <stdexcept>
#include <cstring>
#include <unistd.h>

class CanBridge
{
public:
    CanBridge(const char* Ifname);
    ~CanBridge();
    void send_float(int canid, std::vector<float> txdata_f);
private:
    const char* ifname;
    int sock;
    union Data
    {
    uint32_t data_ui32;
    float data_f32;
    };
};