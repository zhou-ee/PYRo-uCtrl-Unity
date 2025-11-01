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
    _trigger_drv->set_rotate(0.0f);
    _trigger_drv->zero_force();
}

void shoot_17mm_control_t::set_control()
{
    
    if(RC_CONTROL == _total_mode) 
    {
        _last_local_mode = _local_mode;
        if (_local_mode == SHOOT_START)
        {
            return;
        }
        switch(_ready_mode)
        {
        case SHOOT_READY_STOP:
            _local_mode = SHOOT_STOP;
            break;

        case SHOOT_READY_SETUP:
                _local_mode = SHOOT_SETUP;
                if(abs(_fric_drv[0]->get_speed() - _fric_drv[0]->get_target_speed()) < 0.5f &&
                abs(_fric_drv[1]->get_speed() - _fric_drv[1]->get_target_speed()) < 0.5f)
                {
                    _local_mode = SHOOT_READY;
                }
            break;

        case SHOOT_READY_START:
            if(_local_mode == SHOOT_READY)
            {
                _local_mode = SHOOT_START;
                // _is_switch_to_start = true;
            }
            else if(_local_mode == SHOOT_WAIT)
            {
                _local_mode = SHOOT_START;
                // _is_switch_to_start = true;
            }
            break;

        case SHOOT_READY_CONTINUOUS:
            if(_local_mode == SHOOT_WAIT)
            {
                _local_mode = SHOOT_CONTINUOUS;
            }
            break;
        }
        if(_last_local_mode == SHOOT_START && _local_mode != SHOOT_START)
        {
            // _is_switch_to_start = false;
        }

    }

    


    // if(RC_CONTROL == _total_mode)
    // {
    //     if(SHOOT_READY_STOP == _ready_mode)
    //     {
    //         _local_mode = SHOOT_STOP;
    //     }
    //     else if(SHOOT_READY_SETUP == _ready_mode)
    //     {
    //         _local_mode = SHOOT_SETUP;
    //         if(abs(_fric_drv[0]->get_speed() - _fric_drv[0]->get_target_speed()) < 0.5f &&
    //            abs(_fric_drv[1]->get_speed() - _fric_drv[1]->get_target_speed()) < 0.5f)
    //         {
    //             _local_mode = SHOOT_READY;
    //         }
    //     }
    //     else if(SHOOT_READY == _local_mode && SHOOT_READY_START == _ready_mode)
    //     {
    //         _local_mode = SHOOT_START;
    //     }
    //     else if(SHOOT_WAIT == _local_mode && SHOOT_READY_CONTINUOUS == _ready_mode)
    //     {
    //         _local_mode = SHOOT_CONTINUOUS;
    //     }
    // }
}

void shoot_17mm_control_t::control()
{
    update_feedback();
    set_control();

    if(RC_CONTROL == _total_mode)
    {
        switch(_local_mode)
        {
            case SHOOT_STOP:
                zero_force();
                break;

            case SHOOT_SETUP:
            case SHOOT_READY:
                // if(_is_first_setup)
                // {
                //     _first_radian = _trigger_drv->get_radian();
                //     _is_first_setup = false;
                // }
                for(uint8_t i = 0; i < 2; i++)
                {
                    _fric_drv[i]->set_speed(_fric_speed);
                    _fric_drv[i]->control();
                }
                _trigger_drv->set_radian(_trigger_drv->get_radian());
                _trigger_drv->control();
                break;

            case SHOOT_START:
                for(uint8_t i = 0; i < 2; i++)
                {
                    _fric_drv[i]->set_speed(_fric_speed);
                    _fric_drv[i]->control();
                }

                // if(_is_switch_to_start)
                // {
                    float target_total_radian = _trigger_drv->get_radian() - _trigger_drv->get_step_radian();
                    _trigger_drv->set_radian(target_total_radian);
                    // _is_switch_to_start = false;
                // }

                _trigger_drv->control();
                if(abs(_trigger_drv->get_radian() - _trigger_drv->get_target_radian()) < 0.01f)
                {
                    _local_mode = SHOOT_WAIT;
                }          
                break;

            case SHOOT_CONTINUOUS:
                for(uint8_t i = 0; i < 2; i++)
                {
                    _fric_drv[i]->set_speed(_fric_speed);
                    _fric_drv[i]->control();
                }
                _trigger_drv->set_rotate(_trigger_rotate);
                _trigger_drv->control();
                break;

            case SHOOT_WAIT:
                for(uint8_t i = 0; i < 2; i++)
                {
                    _fric_drv[i]->set_speed(_fric_speed);
                    _fric_drv[i]->control();
                }
                _trigger_drv->set_radian(_trigger_drv->get_radian());
                _trigger_drv->control();
                break;
        }
    }
    else if(AUTO_AIM_CONTROL == _total_mode)
    {
        zero_force();
    }
    else
    {
        zero_force();
    }
}
    // float first_radian = 0.0f;
    // if(RC_CONTROL == _total_mode)
    // {
    //     if(SHOOT_STOP == _local_mode)
    //     {
    //         zero_force();
    //         _is_first_setup = true;
    //     }
    //     else if(SHOOT_SETUP == _local_mode || SHOOT_READY == _local_mode)
    //     {
    //         if(_is_first_setup)
    //         {
    //             first_radian = _trigger_drv->get_radian();
    //             _is_first_setup = false;
    //         }
    //         for(uint8_t i = 0; i < 2; i++)
    //         {
    //             _fric_drv[i]->set_speed(_fric_speed);
    //             _fric_drv[i]->control();
    //         }
    //         _trigger_drv->set_radian(first_radian);
    //         _trigger_drv->control();
    //         _is_switch_to_start = true;
    //     }
    //     else if(SHOOT_START == _local_mode)
    //     {
    //         _is_first_setup = true;
    //         for(uint8_t i = 0; i < 2; i++)
    //         {
    //             _fric_drv[i]->set_speed(_fric_speed);
    //             _fric_drv[i]->control();
    //         }
    //         if(_is_switch_to_start)
    //         {
    //             _trigger_drv->set_radian(_trigger_drv->get_radian() - _trigger_drv->get_step_radian());
    //             _is_first_start = false;
    //         }
    //         _trigger_drv->control();
    //         if(abs(_trigger_drv->get_radian() - _trigger_drv->get_target_radian()) < 0.2f)
    //         {
    //             _local_mode = SHOOT_WAIT;
    //         }          
    //     }
    //     else if(SHOOT_CONTINUOUS == _local_mode)
    //     {
    //         _is_switch_to_start = true;
    //         for(uint8_t i = 0; i < 2; i++)
    //         {
    //             _fric_drv[i]->set_speed(_fric_speed);
    //             _fric_drv[i]->control();
    //         }
    //         _trigger_drv->set_rotate(_trigger_rotate);
    //         _trigger_drv->control();
    //         _is_first_setup = true;
    //     }
    // }
    // else if(AUTO_AIM_CONTROL == _total_mode)
    // {
    //     if(SHOOT_STOP == _local_mode)
    //     {
    //         zero_force();
    //     }
    // }
    // else
    // {
    //     zero_force();
    // }
    
}

