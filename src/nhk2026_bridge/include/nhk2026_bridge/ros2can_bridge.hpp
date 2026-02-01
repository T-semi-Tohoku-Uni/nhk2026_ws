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
    void send_int(int canid, std::vector<int> txdata_i);
    void send_bits(int canid, std::vector<bool> txdata_b);
private:
    const char* ifname;
    int sock;
    union Data
    {
    uint32_t data_ui32;
    float data_f32;
    };
};