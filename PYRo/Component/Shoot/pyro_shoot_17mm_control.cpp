#include "pyro_shoot_17mm_control.h"

namespace pyro
{
shoot_17mm_control_t::shoot_17mm_control_t(fric_drv_t *fric_drv_1,
                                           fric_drv_t *fric_drv_2,
                                           trigger_drv_t *trigger_drv)
     : _fric_drv{fric_drv_1, fric_drv_2},
       _trigger_drv(trigger_drv)
{
}

void shoot_17mm_control_t::set_trigger_rotate(float target_rotate)
{
    _trigger_rotate = target_rotate;
}

void shoot_17mm_control_t::set_one_bullet_radian(float radian)
{
    _one_bullet_radian = radian;
}

void shoot_17mm_control_t::set_fric_speed(float target_speed)
{
    _fric_speed = target_speed;
}
    
void shoot_17mm_control_t::update_feedback()
{
    _fric_drv[0]->update_feedback();
    _fric_drv[1]->update_feedback();
    _trigger_drv->update_feedback();
}

void shoot_17mm_control_t::zero_force()
{
    for(uint8_t i = 0; i < 2; i++)
    {
        if(abs(_fric_drv[i]->get_speed()) < 0.5f)
        {
            _fric_drv[i]->zero_force();
        }
        else
        {
            _fric_drv[i]->set_speed(0.0f);
            _fric_drv[i]->control();
        }
    }
}

void shoot_17mm_control_t::set_control()
{
    if(RC_CONTROL == _total_mode)
    {
        if(SHOOT_READY_STOP == _ready_mode)
        {
            _local_mode = SHOOT_STOP;
        }
        else if(SHOOT_READY_SETUP == _ready_mode)
        {
            _local_mode = SHOOT_SETUP;
            if(abs(_fric_drv[0]->get_speed() - _fric_drv[0]->get_target_speed()) < 0.1f &&
               abs(_fric_drv[1]->get_speed() - _fric_drv[0]->get_target_speed()) < 0.1f)
            {
                _local_mode = SHOOT_READY;
            }
        }
        else if(SHOOT_READY == _local_mode && SHOOT_READY_START == _ready_mode)
        {
            _local_mode = SHOOT_START;
            if(abs(_trigger_drv->get_rotate()) > 0.1f)
            {
                _local_mode = SHOOT_WAIT;
            }
        }
        else if(SHOOT_WAIT == _local_mode)
        {
            if(SHOOT_READY_CONTINUOUS == _ready_mode)
            {
                _local_mode = SHOOT_CONTINUOUS;
            }
            else
            {
                _local_mode = SHOOT_READY;
            }
        }
    }
}

void shoot_17mm_control_t::control()
{
    if(RC_CONTROL == _total_mode)
    {
        if(SHOOT_STOP == _local_mode)
        {
            zero_force();
        }
        else if(SHOOT_SETUP == _local_mode || SHOOT_READY == _local_mode)
        {
            for(uint8_t i = 0; i < 2; i++)
            {
                _fric_drv[i]->set_speed(_fric_speed);
                _fric_drv[i]->control();
            }
            _trigger_drv->set_radian(_trigger_drv->get_radian());
            _trigger_drv->control();
        }
        else if(SHOOT_START == _local_mode)
        {
            for(uint8_t i = 0; i < 2; i++)
            {
                _fric_drv[i]->set_speed(_fric_speed);
                _fric_drv[i]->control();
            }
            _trigger_drv->set_radian(_trigger_drv->get_radian() + _one_bullet_radian);
            _trigger_drv->control();
        }
        else if(SHOOT_CONTINUOUS == _local_mode)
        {
            for(uint8_t i = 0; i < 2; i++)
            {
                _fric_drv[i]->set_speed(_fric_speed);
                _fric_drv[i]->control();
            }
            _trigger_drv->set_rotate(_trigger_rotate);
            _trigger_drv->control();
        }
    }
}





}