#include "pyro_pitch_drv.h"

namespace pyro
{
pitch_drv_t::pitch_drv_t(motor_base_t *motor_base,
                const pid_ctrl_t &speed_pid,
                const pid_ctrl_t &position_pid
               )
    : _motor_base(motor_base),
      speed_pid(speed_pid),
      position_pid(position_pid)
{
}

void pitch_drv_t::set_radian(float target_radian)
{
    _target_radian = target_radian;
}

void pitch_drv_t::set_dt(float dt)
{
    _dt = dt;
}

float pitch_drv_t::get_current_radian()
{
    return _current_radian;
}

float pitch_drv_t::get_current_rotate()
{
    return _current_rotate;
}

void pitch_drv_t::zero_force()
{
    _motor_base->send_torque(0.0f);
}

void pitch_drv_t::update_feedback()
{
    _motor_base->update_feedback();
    _current_radian = _motor_base->get_current_position();
    _current_rotate = _motor_base->get_current_rotate();
}

void pitch_drv_t::set_torque(float torque)
{
    _motor_base->send_torque(torque);

}

}

