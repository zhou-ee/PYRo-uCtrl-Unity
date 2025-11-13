#include "pyro_pitch_drv.h"

namespace pyro
{
pitch_drv_t::pitch_drv_t(motor_base_t *motor_base, const pid_ctrl_t &rotate_pid,
                         const pid_ctrl_t &position_pid)
    : motor_base(motor_base), _pitch_rotate_pid(rotate_pid),
      _pitch_position_pid(position_pid)
{
}

void pitch_drv_t::set_offset_radian(float offset_radian)
{
    _offset_radian = offset_radian;
}

void pitch_drv_t::update_target_radian(const float diff_radian)
{
    _target_radian += diff_radian;
    if (_target_radian < 0.05)
    {
        _target_radian = 0.05f;
    }
    else if (_target_radian > 0.4f)
    {
        _target_radian = 0.4f;
    }
}

void pitch_drv_t::calculate_torque(float delta_time)
{
    _target_rotate = _pitch_position_pid.compute(_target_radian,
                                                 _current_radian, delta_time);
    _output_torque =
        _pitch_rotate_pid.compute(_target_rotate, _current_rotate, delta_time);
}

void pitch_drv_t::output_torque() const
{
    motor_base->send_torque(-_output_torque);
}

void pitch_drv_t::zero_force()
{
    _output_torque = 0.0f;
}

void pitch_drv_t::update_feedback(float real_angle, float real_speed)
{
    _current_radian = real_angle - _offset_radian;
    _current_rotate = real_speed;
}

} // namespace pyro