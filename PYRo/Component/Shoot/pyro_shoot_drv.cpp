#include "pyro_shoot_drv.h"

namespace pyro
{
shoot_drv_t::shoot_drv_t(fric_drv_t *fric_drv_1, 
                         fric_drv_t *fric_drv_2, 
                         trigger_drv_t *trigger_drv,
                         rc_drv_t *rc_drv)
    : fric_drv_1(fric_drv_1),
      fric_drv_2(fric_drv_2),
      trigger_drv(trigger_drv)
{
    rc_drv->set_get_mode([this](rc_drv_t *rc_drv) -> void { get_mode(rc_drv); });
}

void shoot_drv_t::get_mode(rc_drv_t *rc_drv)
{
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            rc_drv->get_p_ctrl());
    static auto *p_last_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
        rc_drv->get_p_last_ctrl());
    _s_left = static_cast<uint8_t>(p_ctrl->rc.s[1]);
    _s_left_last = static_cast<uint8_t>(p_last_ctrl->rc.s[1]);
    _s_right = static_cast<uint8_t>(p_ctrl->rc.s[0]);
}

void shoot_drv_t::update_feedback()
{
    fric_drv_1->update_feedback();
    fric_drv_2->update_feedback();
    trigger_drv->update_feedback();
}

void shoot_drv_t::zero_force()
{
    fric_drv_1->zero_force();
    fric_drv_2->zero_force();
    trigger_drv->zero_force();
}

void shoot_drv_t::shoot_control()
{
    if(_s_right == 1)
    {
        zero_force();
    }
    else
    {
        if(_s_left == 1)
        {
         zero_force();
        }
        else if(_s_left == 3 && _s_left_last != 2)
        {
            fric_drv_1->fric_control();
            fric_drv_2->fric_control();
            trigger_drv->zero_force();
        }
        else if(_s_left == 3 && _s_left_last == 2)
        {
            fric_drv_1->fric_control();
            fric_drv_2->fric_control();
            trigger_drv->step_fire();
        }
        else if(_s_left == 2 && _s_left_last == 2)
        {
            fric_drv_1->fric_control();
            fric_drv_2->fric_control();
            trigger_drv->continue_fire();
        }
    }
    
}

}