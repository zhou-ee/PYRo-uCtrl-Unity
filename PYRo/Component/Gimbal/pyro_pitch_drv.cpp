/*
 * @Author: lucky zhou_20006ee@163.com
 * @Date: 2025-11-13 22:05:21
 * @LastEditors: lucky zhou_20006ee@163.com
 * @LastEditTime: 2025-11-14 02:24:27
 * @FilePath: \PYRo-uCtrl-Unity\PYRo\Component\Gimbal\pyro_pitch_drv.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "pyro_pitch_drv.h"

#include <utility>

namespace pyro
{
pitch_drv_t::pitch_drv_t(motor_base_t *motor_base, pid_t rotate_pid,
                         pid_t position_pid)
    : motor_base(motor_base), _pitch_rotate_pid(std::move(rotate_pid)),
      _pitch_position_pid(std::move(position_pid))
{
}

void pitch_drv_t::set_offset_radian(float offset_radian)
{
    _offset_radian = offset_radian;
}

void pitch_drv_t::update_target_radian(const float diff_radian)
{
    _target_radian += diff_radian;
    if (_target_radian < -0.6f)
    {
        _target_radian = -0.6f;
    }
    else if (_target_radian > -0.1f)
    {
        _target_radian = -0.1f;
    }
}

void pitch_drv_t::calculate_torque()
{
    _target_rotate =
        _pitch_position_pid.calculate(_current_radian, _target_radian);
    _output_torque =
        _pitch_rotate_pid.calculate(_current_rotate, _target_rotate);
}

void pitch_drv_t::output_torque() const
{
    motor_base->send_torque(_output_torque);
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