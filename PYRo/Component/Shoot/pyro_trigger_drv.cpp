#include "pyro_trigger_drv.h"
#include "cmsis_os.h"

#define BLOCK_THRESHOLD 400
#define BLOCK_SPEED 0.3f
#define BLOCK_TIME 10

namespace pyro
{

trigger_drv_t::trigger_drv_t(motor_base_t *motor_base,
                             const pid_ctrl_t &_rotate_pid, 
                             const pid_ctrl_t &_position_pid,
                             float step_radian
                     )
    : motor_base(motor_base),
      _rotate_pid(_rotate_pid),
      _position_pid(_position_pid),
      _step_radian(step_radian)
{
    update_feedback();
    _target_trigger_total_radian = _current_trigger_total_radian;
}

void trigger_drv_t::set_dt(float dt)
{
    _dt = dt;
}

void trigger_drv_t::set_gear_ratio(float gear_ratio)
{
    _gear_ratio = gear_ratio;
}

void trigger_drv_t::set_rotate(float target_rotate)
{
    if (POSITION == _mode)
    {
        _rotate_pid.reset();
        _position_pid.reset();
    }
    _mode = ROTATE;
    _target_trigger_rotate = -target_rotate;
}

void trigger_drv_t::set_radian(float target_radian)
{
    if (ROTATE == _mode)
    {
        _rotate_pid.reset();
    }
    _mode = POSITION;

    _target_trigger_total_radian = target_radian;

    // const float full_circle = 2.0f * PI;
    // if (target_radian > PI)
    // {
    //     target_radian -= full_circle;
    // }
    // else if (target_radian < -PI)
    // {
    //     target_radian += full_circle;
    // }
    // _target_trigger_radian = target_radian;

}

void trigger_drv_t::step_forward()
{
    _target_trigger_total_radian = _current_trigger_total_radian - _step_radian;
}

float trigger_drv_t::get_rotate()
{
    return _current_trigger_rotate;
}

float trigger_drv_t::get_radian()
{
    return _current_trigger_total_radian;
}

float trigger_drv_t::get_target_radian()
{
    return _target_trigger_total_radian;
}

float trigger_drv_t::get_step_radian()
{
    return _step_radian;
}

void trigger_drv_t::zero_force()
{
    motor_base->send_torque(0.0f);
}

void trigger_drv_t::update_feedback()
{
    motor_base->update_feedback();
    _current_trigger_rotate = motor_base->get_current_rotate() / _gear_ratio;
    _last_motor_radian = _current_motor_radian;
    _current_motor_radian = motor_base->get_current_position();
    _motor_to_trigger_radian();
}

void trigger_drv_t::control()
{
    update_feedback();
    if(ROTATE == _mode)
    {
        float torque_cmd = _rotate_pid.compute(_target_trigger_rotate, _current_trigger_rotate, _dt);
        motor_base->send_torque(torque_cmd);
    }
    else if(POSITION == _mode) 
    {
            float rotate_cmd = _position_pid.compute(_target_trigger_total_radian, _current_trigger_total_radian, _dt);

_test_rotate_cmd = rotate_cmd;

            float torque_cmd = _rotate_pid.compute(rotate_cmd, _current_trigger_rotate, _dt);
_test_torque_cmd = torque_cmd;
            motor_base->send_torque(torque_cmd);
    }
}

void trigger_drv_t::_motor_to_trigger_radian()
{
    float delta_motor = _current_motor_radian - _last_motor_radian;
    const float full_circle = 2.0f * PI;
    if (delta_motor < -PI)
    {
        delta_motor += full_circle;
    }
    else if (delta_motor > PI)
    {
        delta_motor -= full_circle;
    }

    _current_trigger_total_radian += delta_motor / _gear_ratio;
    // const float full_circle = 2.0f * PI;
    // if (delta_motor < -PI)
    // {
    //     delta_motor += full_circle;
    // }
    // else if (delta_motor > PI)
    // {
    //     delta_motor -= full_circle;
    // }

    // _current_trigger_total_radian += delta_motor / _gear_ratio;

    // _current_trigger_radian += delta_motor / _gear_ratio;
    // if(_current_trigger_radian > PI)
    // {
    //     _current_trigger_radian -= full_circle;
    // }
    // if(_current_trigger_radian < -PI)
    // {
    //     _current_trigger_radian += full_circle;
    // }
}

// float trigger_drv_t::_update_trigger_radian()
// {
//     if(_is_first_update)
//     {
//         _last_motor_radian = _current_motor_radian;
//         _is_first_update = false;
//         return _last_motor_radian / _gear_ratio;
//     }
    
//     float delta_motor_radian = _current_motor_radian - _last_motor_radian;
//     if(delta_motor_radian < -PI)
//     {
//         _motor_total_count += 1;
//         delta_motor_radian += 2 * PI;
//     }
//     else if(delta_motor_radian > PI)
//     {
//         _motor_total_count -= 1;
//         delta_motor_radian -= 2 * PI;
//     }

//     _last_motor_radian = _current_motor_radian;

//     float total_motor_radian = _current_motor_radian + _motor_total_count * 2 * PI;
//     float gear_radian = total_motor_radian / _gear_ratio;
//     if(gear_radian < -PI)
//         gear_radian += 2 * PI;
//     else if(gear_radian > PI)
//         gear_radian -= 2 * PI;

//     return gear_radian;
// }

}