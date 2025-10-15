#include "pyro_motor_base.h"



pyro::motor_base_t::motor_base_t(pyro::can_hub_t::which_can which)
    : _which_can(which), _enable(false), _temperature(0), _current_position(0),
      _current_rotate(0), _current_torque(0)
{
    // pyro::pyro_can_hub_t::get_instance()->hub_register_can_obj(hub.get_hfdcan(),this);
    _can_drv = pyro::can_hub_t::get_instance()->hub_get_can_obj(which);
    //_feedback_msg = std::make_unique<pyro::can_msg_buffer_t>(id);
    // if(_can_drv)
    // {
    //     _can_drv->status_t(_feedback_msg.get());
    // }
}

int8_t pyro::motor_base_t::get_temperature(void)
{
    return _temperature;
}

float pyro::motor_base_t::get_current_position(void)
{
    return _current_position;
}

float pyro::motor_base_t::get_current_rotate(void)
{
    return _current_rotate;
}

float pyro::motor_base_t::get_current_torque(void)
{
    return _current_torque;
}

bool pyro::motor_base_t::is_enable(void)
{
    return _enable;
}