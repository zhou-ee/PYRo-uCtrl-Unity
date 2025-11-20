#include "pyro_powermeter.h"
#include <cstring>

namespace pyro {

powermeter_drv_t::powermeter_drv_t(uint32_t can_id, can_hub_t::which_can which)
    : _can_id(can_id),
      _which(which),  // 保存CAN集线器引用
      _can_drv(nullptr),
      _msg_buffer(nullptr),
      _data_updated(true) 
{
    _latest_data = {0.0f, 0.0f, 0.0f};
}

status_t powermeter_drv_t::init() 
{
    can_hub_t* can_hub = can_hub_t::get_instance();
    if (can_hub  == nullptr) {
        return PYRO_ERROR;
    }

    _can_drv = can_hub->hub_get_can_obj(_which);
    if (_can_drv == nullptr) {
        return PYRO_ERROR;
    }

    _msg_buffer = new can_msg_buffer_t(_can_id);
    if (_msg_buffer == nullptr) 
    {
        return PYRO_NO_MEMORY;
    }

    status_t reg_status = _can_drv->register_rx_msg(_msg_buffer);
    if (reg_status != PYRO_OK)
    {
        delete _msg_buffer;
        _msg_buffer = nullptr;
        return reg_status;
    }

    return PYRO_OK;
}

bool powermeter_drv_t::get_data(powermeter_data& data) {
    if (!_data_updated || _msg_buffer == nullptr) 
    {
        return false;
    }

    std::array<uint8_t, 8> raw_data;
    if (_msg_buffer->get_data(raw_data)) 
    {
        _data_updated = false;
        process_can_data(raw_data);
        data = _latest_data;
        _msg_buffer->mark_read();
        return true;
    }

    return false;
}

void powermeter_drv_t::process_can_data(const std::array<uint8_t, 8>& data) 
{
    int16_t raw_current = (int16_t)((data[1] << 8) | data[0]);
    _latest_data.current = static_cast<float>(raw_current) / 100.0f;

    int16_t raw_voltage = (int16_t)((data[3] << 8) | data[2]);
    _latest_data.voltage = static_cast<float>(raw_voltage) / 100.0f;

    _latest_data.power = _latest_data.current * _latest_data.voltage;

    _data_updated = true;
}

}
