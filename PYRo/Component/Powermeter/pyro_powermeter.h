#ifndef __POWERMETER_H__
#define __POWERMETER_H__

#include "pyro_can_drv.h"
#include <array>

namespace pyro {

struct powermeter_data {
    float current;
    float voltage;
    float power;
};

class powermeter_drv_t {
public:
    powermeter_drv_t(uint32_t can_id, can_hub_t::which_can which);
    ~powermeter_drv_t()
    {
    }

    status_t init();

    bool get_data(powermeter_data& data);

private:
    uint32_t _can_id;
    can_hub_t::which_can _which;    
    can_drv_t* _can_drv;
    can_msg_buffer_t* _msg_buffer;
    powermeter_data _latest_data;
    volatile bool _data_updated;

    void process_can_data(const std::array<uint8_t, 8>& data);
};

}

#endif

