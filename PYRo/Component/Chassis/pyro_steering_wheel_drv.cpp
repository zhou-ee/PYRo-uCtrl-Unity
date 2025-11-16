#include "pyro_steering_wheel_drv.h"

static inline float wrap_rad(float x)
{
    return atan2f(sinf(x), cosf(x));
}

namespace pyro
{

float wrap_pi(float x)
{
    while (x > PI) x -= 2 * PI;
    while (x <= -PI) x += 2 * PI;
    return x;
}

static float loop_float_constrain(float Input, float minValue, float maxValue);

steering_wheel_drv_t::steering_wheel_drv_t(wheel_drv_t *wheel_drv,
                                           motor_base_t *rudder_motor_base,
                                           const pid_t &rudder_rotate_pid,
                                           const pid_t &rudder_position_pid)
    : wheel_drv(wheel_drv),
      rudder_motor_base(rudder_motor_base),
      _rudder_rotate_pid(rudder_rotate_pid),
      _rudder_position_pid(rudder_position_pid)
{
}

void steering_wheel_drv_t::set_offset_radian(float offset_radian)
{
    _offset_radian = offset_radian;
}

void steering_wheel_drv_t::set_radian(float target_radian)
{
    // 1. 修正目标角：带偏移并规约到 (-π, π]
    target_radian = wrap_pi(target_radian);

    // 2. 计算两种等价目标（不在这里wrap，以免跳变）
    float target_a = target_radian;      // 正向
    float target_b = target_radian + PI; // 反向候选（暂不规约）

    // 3. 确保两者相对于当前角的差是连续的
    float delta_a = wrap_pi(target_a - _current_radian);
    float delta_b = wrap_pi(target_b - _current_radian);

    // 4. 比较两者的绝对值
    float abs_a = fabsf(delta_a);
    float abs_b = fabsf(delta_b);

    // 滞回参数（防止切换）
    const float hysteresis = 0.15f; // 约9°
    float chosen_target;

    static uint8_t _chosen_mode;

    // 5. 判断选哪一个（并保持状态避免反复）
    if (_chosen_mode == 0 && abs_b < abs_a - hysteresis)
    {
        _chosen_mode = 1;
    }
    else if (_chosen_mode == 1 && abs_a < abs_b - hysteresis)
    {
        _chosen_mode = 0;
    }

    chosen_target = _chosen_mode == 0 ? target_a : target_b;

    if (_chosen_mode == 0)
    {
        chosen_target = target_a;
        direction = 1;
    }
    else
    {
        chosen_target = target_b;
        direction = -1;
    }

    // 6. 再 wrap 一次最终目标差，得到连续误差
    float angle_error = wrap_pi(chosen_target - _current_radian);

    // 7. 控制律
    float rotate_cmd = _rudder_position_pid.calculate(angle_error + _current_radian, _current_radian);
    float torque_cmd = _rudder_rotate_pid.calculate(rotate_cmd, rudder_motor_base->get_current_rotate());

    rudder_motor_base->send_torque(torque_cmd);
}

void steering_wheel_drv_t::zero_force()
{
    rudder_motor_base->send_torque(0.0f);
    wheel_drv->zero_force();
}

void steering_wheel_drv_t::update_feedback()
{
    wheel_drv->update_feedback();
    rudder_motor_base->update_feedback();
    _current_radian = loop_float_constrain(rudder_motor_base->get_current_position() - _offset_radian, -PI, PI);

    if (_current_radian > PI || _current_radian < -PI)
    {
        while (1);
    }

}

float steering_wheel_drv_t::get_target_radian()
{
    return _target_radian;
}

//循环限幅函数
static float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}


}
