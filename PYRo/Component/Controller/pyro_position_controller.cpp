#include "pyro_position_controller.h"

namespace pyro
{

position_controller_t::position_controller_t(motor_base_t* motor, pid_ctrl_t* pos_pid, pid_ctrl_t* rot_pid)
    : closed_controller_t(motor), _pos_pid(pos_pid), _rot_pid(rot_pid)
{
    _target_pos = 0.0f;
    _target_rot = 0.0f;
    _feedback_pos = 0.0f;
    _feedback_rot = 0.0f;
    _control_value = 0.0f;
}

float position_controller_t::set_target(float target)
{
    _target_pos = target;
    if(_constraint == CONSTRAINT)
    {
        if(_target_pos > _upper_limit)
        {
            _target_pos = _upper_limit;
        }
        else if(_target_pos < _lower_limit)
        {
            _target_pos = _lower_limit;
        }
    }
    // if(_target_pos > _upper_limit)
    // {
    //     _target_pos = _upper_limit;
    // }
    // else if(_target_pos < _lower_limit)
    // {
    //     _target_pos = _lower_limit;
    // }
    return _target_pos;
}

void position_controller_t::update()
{
    _motor->update_feedback();
    _feedback_pos = _motor->get_current_position();
    _feedback_rot = _motor->get_current_rotate();
}
static float angle_correction(float target, float feedback,float max)
{
    if(target - feedback > max)
    {
        return target -2 * max;
    }
    else if(feedback - target > max)
    {
        return target +2 * max;
    }
    else
    {
        return target;
    }
}
void position_controller_t::control(float dt)
{
    _target_rot = _pos_pid->compute(angle_correction(_target_pos, _feedback_pos, pyro::PI), _feedback_pos, dt);
    _control_value = _rot_pid->compute(_target_rot, _feedback_rot, dt);
    status_t ret = _motor->send_torque(_control_value);
}

void position_controller_t::set_upper_limit(float upper_limit)
{
    _upper_limit = upper_limit;
}

void position_controller_t::set_lower_limit(float lower_limit)
{
    _lower_limit = lower_limit;
}

};