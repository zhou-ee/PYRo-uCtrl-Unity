#include "pyro_fric_drv.h"

namespace pyro
{
fric_drv_t::fric_drv_t(motor_base_t *motor_base,
                     const pid_ctrl_t &speed_pid, float radius)
    : motor_base(motor_base),
      _speed_pid(speed_pid),
      _radius(radius)
{
};

void fric_drv_t::set_speed(float target_speed)
{
    _target_speed = target_speed;
}

void fric_drv_t::zero_force()
{
    if(abs(_current_speed) < 0.5f)
    {
        motor_base->send_torque(0.0f);
    }
    else
    {
        float torque_cmd = _speed_pid.compute(0.0f,
                    _current_speed, 0.001f);
        motor_base->send_torque(torque_cmd);
    }
    // motor_base->send_torque(0.0f);
}

void fric_drv_t::fric_control()
{
    float torque_cmd = _speed_pid.compute(_target_speed, _current_speed, 0.001f);
    motor_base->send_torque(torque_cmd);
}

void fric_drv_t::update_feedback()
{
    motor_base->update_feedback();
    _current_speed = motor_base->get_current_rotate() * _radius;
}

}
