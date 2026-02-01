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
#include <string>

class CanBridge
{
public:
    struct RxData_struct
    {
        int canid;
        std::vector<uint8_t> data;
    };

    CanBridge(const std::string Ifname);
    ~CanBridge();
    void send_float(int canid, std::vector<float> txdata_f);
    void send_int(int canid, std::vector<int> txdata_i);
    void send_bits(int canid, std::vector<bool> txdata_b);
    RxData_struct receive_data();
     
private:
    const std::string ifname;
    int sock;
    union Data
    {
        uint32_t data_ui32;
        float data_f32;
    };
};