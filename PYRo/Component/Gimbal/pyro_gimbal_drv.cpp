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
    rc_drv_t *dr16_drv = rc_hub_t::get_instance(rc_hub_t::which_rc_t::DR16);
    dr16_drv->config_rc_cmd([this](rc_drv_t *rc_drv) -> void
    {
        this->dr16_cmd();
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
}

void gimbal_drv_t::zero_force()
{ 
    _pitch_drv->zero_force();
    _roll_drv->zero_force();
    _yaw_drv->zero_force();
}

void gimbal_drv_t::dr16_cmd()
{
    rc_drv_t *dr16_drv = rc_hub_t::get_instance(rc_hub_t::which_rc_t::DR16);
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            dr16_drv->get_p_ctrl());
    if(dr16_drv_t::DR16_SW_UP == static_cast<uint8_t>(p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT]))
    {
        _total_mode = ZERO_FORCE;
    }
    else if(dr16_drv_t::DR16_SW_MID == static_cast<uint8_t>(p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT]))
    {
        _total_mode = RC_CONTROL;
    }
    else if(dr16_drv_t::DR16_SW_DOWN == static_cast<uint8_t>(p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT]))
    {
        _total_mode = AUTO_AIM_CONTROL;
    }
}

void gimbal_drv_t::vt03_cmd()
{
}

void gimbal_drv_t::set_control()
{
    if(_total_mode == ZERO_FORCE)
    {
        
    }
    else if(_total_mode == RC_CONTROL)
    {
        
    }
    else if(_total_mode == AUTO_AIM_CONTROL)
    {
        
    }
    else
    {
        
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
        _pitch_drv->zero_force();
        _roll_drv->zero_force();
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





