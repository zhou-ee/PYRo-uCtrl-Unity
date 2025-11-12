#include "pyro_gimbal_drv.h"

namespace pyro
{
gimbal_drv_t::gimbal_drv_t(pitch_drv_t *pitch_drv, 
                           roll_drv_t *roll_drv, 
                           yaw_drv_t *yaw_drv,
                           IMU_obj *imu)
    : _pitch_drv(pitch_drv),
      _roll_drv(roll_drv),
      _yaw_drv(yaw_drv),
      _imu(imu)
{
    pyro::rc_hub_t::get_instance(
        pyro::rc_hub_t::DR16)->config_rc_cmd([this](void const *rc_ctrl)->void{
            return dr16_cmd(rc_ctrl);
        });
    pyro::rc_hub_t::get_instance(
        pyro::rc_hub_t::VT03)->config_rc_cmd([this](void const *rc_ctrl)->void{
            return vt03_cmd(rc_ctrl);
        });
}

void gimbal_drv_t::set_dt(float dt)
{
    _dt = dt;
}

void gimbal_drv_t::update_feedback()
{ 
    _pitch_drv->update_feedback();
    _roll_drv->update_feedback();
    _yaw_drv->update_feedback();
    if(_is_init)
    {
        while(_imu_yaw_offset_radian == 0.0f)
        {
            _yaw_drv->update_feedback();

            if(_yaw_drv->get_current_radian() < 0.0f)
            {
                _imu_yaw_offset_radian = -(_yaw_drv->get_current_radian() + 2.0f * PI);
            }
            else
            {
                _imu_yaw_offset_radian = -_yaw_drv->get_current_radian();
            }

        }
        _target_yaw_imu_radian = _imu_yaw_offset_radian;
        _is_init = false;
    }
    
    _current_yaw_imu_radian = _imu->Get_Yaw(_imu) + _imu_yaw_offset_radian;
    _current_pitch_imu_radian = _imu->Get_Pitch(_imu);
    _current_roll_imu_radian = _imu->Get_Roll(_imu);
}

void gimbal_drv_t::zero_force()
{ 
    _pitch_drv->zero_force();
    _roll_drv->zero_force();
    _yaw_drv->zero_force();
}

void gimbal_drv_t::dr16_cmd(void const *rc_ctrl)
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

    if(RC_CONTROL == _total_mode)
    { 
        _target_yaw_imu_radian -= static_cast<float>(p_ctrl->rc.ch[dr16_drv_t::DR16_CH_RIGHT_X]) * 0.1f;
        if(_target_yaw_imu_radian > -2.2f)
        {
            _target_yaw_imu_radian = -2.2f;
        }
        else if(_target_yaw_imu_radian < -5.8f)
        {
            _target_yaw_imu_radian = -5.8f;
        }

        _target_pitch_imu_radian += static_cast<float>(p_ctrl->rc.ch[dr16_drv_t::DR16_CH_RIGHT_Y]) * 0.1f;
        if(_target_pitch_imu_radian < -0.5f)
        {
            _target_pitch_imu_radian = -0.5f;
        }
        else if(_target_pitch_imu_radian > 0.13f)
        {
            _target_pitch_imu_radian = 0.13f;
        }
    }

}

void gimbal_drv_t::vt03_cmd(void const *rc_ctrl)
{
    static auto *p_ctrl =
        static_cast<pyro::vt03_drv_t::vt03_ctrl_t const *>(rc_ctrl);
    if(vt03_drv_t::VT03_GEAR_RIGHT == p_ctrl->rc.gear.state)
    {
        _total_mode = ZERO_FORCE;
    }
    else if(vt03_drv_t::VT03_GEAR_MID == p_ctrl->rc.gear.state)
    {
        _total_mode = RC_CONTROL;
    }
    else if(vt03_drv_t::VT03_GEAR_LEFT == p_ctrl->rc.gear.state)
    {
        _total_mode = AUTO_AIM_CONTROL;
    }

    if(RC_CONTROL == _total_mode)
    { 
        if(0 == p_ctrl->rc.ch[vt03_drv_t::VT03_CH_RIGHT_Y] && 0 == p_ctrl->rc.ch[vt03_drv_t::VT03_CH_RIGHT_X])
        { 
            _target_yaw_imu_radian -= static_cast<float>(p_ctrl->mouse.y) * 0.1f;
            if(_target_yaw_imu_radian > -2.2f)
            {
                _target_yaw_imu_radian = -2.2f;
            }
            else if(_target_yaw_imu_radian < -5.8f)
            {
                _target_yaw_imu_radian = -5.8f;
            }

            _target_pitch_imu_radian += static_cast<float>(p_ctrl->mouse.x) * 0.1f;
            if(_target_pitch_imu_radian < -0.5f)
            {
                _target_pitch_imu_radian = -0.5f;
            }
            else if(_target_pitch_imu_radian > 0.13f)
            {
                _target_pitch_imu_radian = 0.13f;
            }
        }
        else
        {
            _target_yaw_imu_radian -= static_cast<float>(p_ctrl->rc.ch[vt03_drv_t::VT03_CH_RIGHT_X]) * 0.1f;
            if(_target_yaw_imu_radian > -2.2f)
            {
                _target_yaw_imu_radian = -2.2f;
            }
            else if(_target_yaw_imu_radian < -5.8f)
            {
                _target_yaw_imu_radian = -5.8f;
            }

            _target_pitch_imu_radian += static_cast<float>(p_ctrl->rc.ch[vt03_drv_t::VT03_CH_RIGHT_Y]) * 0.1f;
            if(_target_pitch_imu_radian < -0.5f)
            {
                _target_pitch_imu_radian = -0.5f;
            }
            else if(_target_pitch_imu_radian > 0.13f)
            {
                _target_pitch_imu_radian = 0.13f;
            }
        }
    }

}

void gimbal_drv_t::set_control()
{
    if(_total_mode == ZERO_FORCE)
    {
        zero_force();
    }
    else if(_total_mode == RC_CONTROL)
    {
        _yaw_speed_cmd = _yaw_drv->position_pid.compute(_target_yaw_imu_radian, _current_yaw_imu_radian, _dt);
        _yaw_cmd_torque = _yaw_drv->speed_pid.compute(-_yaw_speed_cmd, _yaw_drv->get_current_rotate(), _dt);

        float pitch_speed_cmd = _pitch_drv->position_pid.compute(_target_pitch_imu_radian, _current_pitch_imu_radian, _dt);
        _pitch_cmd_torque = _pitch_drv->speed_pid.compute(pitch_speed_cmd, _pitch_drv->get_current_rotate(), _dt);

        float roll_speed_cmd = _pitch_drv->position_pid.compute(0.0f, _current_roll_imu_radian, _dt);
        _roll_cmd_torque = _pitch_drv->speed_pid.compute(roll_speed_cmd, _roll_drv->get_current_rotate(), _dt);
    }
    else if(_total_mode == AUTO_AIM_CONTROL)
    {
        zero_force();
    }
    else
    {
        zero_force();
    }
}

void gimbal_drv_t::control()
{
    if(_total_mode == ZERO_FORCE)
    {
        zero_force();
    }
    else if(_total_mode == RC_CONTROL)
    {
        _yaw_drv->set_torque(_yaw_cmd_torque);
        _pitch_drv->set_torque(_pitch_cmd_torque);
        _roll_drv->set_torque(_roll_cmd_torque);
    }
    else if(_total_mode == AUTO_AIM_CONTROL)
    {
        zero_force();
    }
    else
    {
        zero_force();   
    }
}

}





