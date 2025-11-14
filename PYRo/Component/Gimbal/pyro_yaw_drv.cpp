#include "pyro_yaw_drv.h"

namespace pyro
{
yaw_drv_t::yaw_drv_t(motor_base_t *motor_base,
                const pid_ctrl_t &rotate_pid,
                const pid_ctrl_t &position_pid,
                IMU_obj *imu)
    : motor_base(motor_base),
      _yaw_rotate_pid(rotate_pid),
      _yaw_position_pid(position_pid),
      _imu(imu)
{
    _IMU_zero_point = imu->Get_Yaw(_imu);
    pyro::rc_hub_t::get_instance(
        pyro::rc_hub_t::DR16)->config_rc_cmd([this](void const *rc_ctrl)->void{
            return dr16_cmd(rc_ctrl);
        });
    pyro::rc_hub_t::get_instance(
        pyro::rc_hub_t::VT03)->config_rc_cmd([this](void const *rc_ctrl)->void{
            return vt03_cmd(rc_ctrl);
        });
}

void yaw_drv_t::set_offset_radian(float offset_radian)
{
    _offset_radian = offset_radian;
}

void yaw_drv_t::set_radian(float target_radian)
{
    if(_current_radian < 0)
    {
        _current_radian += 2*PI;
    }

    if(target_radian < 0)
    {
        target_radian += 2*PI;
    }

    float radian_diff = target_radian - _current_radian;

    if(radian_diff > PI)
    {
        radian_diff -= 2 * PI;
    }
    else if (radian_diff < -PI)
    {
        radian_diff += 2 * PI;
    }

    float rotate_cmd =
        _yaw_position_pid.compute(_current_radian + radian_diff, _current_radian, 0.001f);
    float torque_cmd =
        _yaw_rotate_pid.compute(rotate_cmd, motor_base->get_current_rotate(), 0.001f);
    motor_base->send_torque(torque_cmd);
}

void yaw_drv_t::zero_force()
{
    motor_base->send_torque(0.0f);
}

void yaw_drv_t::dr16_cmd(void const *rc_ctrl)
{
    static auto *p_ctrl =
        static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
    if(dr16_drv_t::DR16_SW_UP == p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT].state)
    {
        _total_mode = ZERO_FORCE;
    }
    else if(dr16_drv_t::DR16_SW_MID == p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT].state)
    {
        _total_mode = RC_CONTROL;
    }
    else if(dr16_drv_t::DR16_SW_DOWN == p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT].state)
    {
        _total_mode = AUTO_AIM_CONTROL;
    }
    if(ZERO_FORCE == _total_mode)
    {
        _target_yaw_imu_radian = _current_yaw_imu_radian;
    }
    else if(RC_CONTROL == _total_mode || AUTO_AIM_CONTROL == _total_mode)
    { 
        _target_yaw_imu_radian -= static_cast<float>(p_ctrl->rc.ch[dr16_drv_t::DR16_CH_RIGHT_X]) * 0.1f;
        if(_target_yaw_imu_radian - _current_yaw_imu_radian > PI)
        {
            _target_yaw_imu_radian -= 2 * PI;
        }
        else if(_target_yaw_imu_radian - _current_yaw_imu_radian < -PI)
        {
            _target_yaw_imu_radian += 2 * PI;
        }
    }
}

void yaw_drv_t::vt03_cmd(void const *rc_ctrl)
{
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


void yaw_drv_t::update_feedback()
{
    motor_base->update_feedback();
    _current_rotate = motor_base->get_current_rotate();
    _current_yaw_imu_radian = _imu->Get_Yaw(_imu);
    _current_radian = loop_float_constrain(motor_base->get_current_position() - _offset_radian, -PI, PI);
    if (_current_radian > PI || _current_radian < -PI)
    {
        while (1);
    }
}

float yaw_drv_t::get_yaw_motor_err()
{
    _yaw_motor_err = _imu->Get_Yaw(_imu) - _IMU_zero_point;
    return _yaw_motor_err;
}

float yaw_drv_t::get_target_radian()
{
    return _target_radian;
}
float yaw_drv_t::get_radian()
{
    return _current_radian;
}

void yaw_drv_t::set_control()
{
    if(_total_mode == ZERO_FORCE)
    {
        zero_force();
    }
    else if(_total_mode == RC_CONTROL || _total_mode == AUTO_AIM_CONTROL)
    {
        float _yaw_speed_cmd = _yaw_position_pid.compute(_target_yaw_imu_radian, _current_yaw_imu_radian, 0.001f);
        _yaw_cmd_torque = _yaw_rotate_pid.compute(_yaw_speed_cmd, _current_rotate, 0.001f);
    }
    else
    {
        zero_force();
    }
}

void yaw_drv_t::control()
{
    if(_total_mode == ZERO_FORCE)
    {
        zero_force();
    }
    else if(_total_mode == RC_CONTROL || _total_mode == AUTO_AIM_CONTROL)
    {
        motor_base->send_torque(_yaw_cmd_torque);
    }
    else
    {
        zero_force();   
    }
}

}