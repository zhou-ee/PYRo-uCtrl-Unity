#include "pyro_macunum_chassis_drv.h"

#include "pyro_core_def.h"

#include <cmath>

namespace pyro
{
    macunum_chassis_drv_t::macunum_chassis_drv_t(wheel_drv_t* wheel_1, wheel_drv_t* wheel_2, wheel_drv_t* wheel_3, wheel_drv_t* wheel_4, macunum_type type,float _rx,float _ry)
    : _wheel_1(wheel_1), _wheel_2(wheel_2), _wheel_3(wheel_3), _wheel_4(wheel_4), _type(type),_rx(_rx),_ry(_ry){}


    void macunum_chassis_drv_t::set_target(float vx, float vy, float wz)
    {
        _target_vx = vx; 
        _target_vy = vy;
        _target_wz = wz;
    }

    void macunum_chassis_drv_t::update()
    {
        _wheel_1->update();
        _wheel_2->update();
        _wheel_3->update();
        _wheel_4->update();

        if(_type == TYPE_X)
        {

        }
        else
        {
            
        }
    }


    void macunum_chassis_drv_t::control(float dt)
    {
        static float forward_angle [4] = {0.0f, PI, PI, 0.0f};// gamma
        static float roller_angle [4] = {-PI/4, -3*PI/4, 3*PI/4, PI/4};// X side // sita

        float rx_sign[4] = {1.0f, 1.0f, -1.0f, -1.0f};
        float ry_sign[4] = {1.0f, -1.0f, -1.0f, 1.0f};

        float vx = _target_vx;
        float vy = _target_vy;
        float wz = _target_wz;

        float v_wheel [4] = {0.0f, 0.0f, 0.0f, 0.0f};

        if(_type == TYPE_X)
        {
             _wheel_spd[0]=(cosf(roller_angle[0])*(vx-wz*_ry*ry_sign[0])+sinf(roller_angle[0])*(vy+wz*_rx*rx_sign[0]))/cosf(forward_angle[0]-roller_angle[0])/_wheel_1->_radius;
             _wheel_spd[1]=(cosf(roller_angle[1])*(vx-wz*_ry*ry_sign[1])+sinf(roller_angle[1])*(vy+wz*_rx*rx_sign[1]))/cosf(forward_angle[1]-roller_angle[1])/_wheel_2->_radius;
             _wheel_spd[2]=(cosf(roller_angle[2])*(vx-wz*_ry*ry_sign[2])+sinf(roller_angle[2])*(vy+wz*_rx*rx_sign[2]))/cosf(forward_angle[2]-roller_angle[2])/_wheel_3->_radius;
             _wheel_spd[3]=(cosf(roller_angle[3])*(vx-wz*_ry*ry_sign[3])+sinf(roller_angle[3])*(vy+wz*_rx*rx_sign[3]))/cosf(forward_angle[3]-roller_angle[3])/_wheel_4->_radius;
        }
        else
        {
            _wheel_spd[0]=(cosf(roller_angle[3])*(vx-wz*_ry*ry_sign[0])+sinf(roller_angle[3])*(vy+wz*_rx*rx_sign[0]))/cosf(forward_angle[0]-roller_angle[3])/_wheel_1->_radius;
             _wheel_spd[1]=(cosf(roller_angle[2])*(vx-wz*_ry*ry_sign[1])+sinf(roller_angle[2])*(vy+wz*_rx*rx_sign[1]))/cosf(forward_angle[1]-roller_angle[2])/_wheel_2->_radius;
             _wheel_spd[2]=(cosf(roller_angle[1])*(vx-wz*_ry*ry_sign[2])+sinf(roller_angle[1])*(vy+wz*_rx*rx_sign[2]))/cosf(forward_angle[2]-roller_angle[1])/_wheel_3->_radius;
             _wheel_spd[3]=(cosf(roller_angle[0])*(vx-wz*_ry*ry_sign[3])+sinf(roller_angle[0])*(vy+wz*_rx*rx_sign[3]))/cosf(forward_angle[3]-roller_angle[0])/_wheel_4->_radius;
        }

        _wheel_1->set_target( _wheel_spd[0]);
        _wheel_2->set_target( _wheel_spd[1]);
        _wheel_3->set_target( _wheel_spd[2]);
        _wheel_4->set_target( _wheel_spd[3]);

        _wheel_1->control(dt);
        _wheel_2->control(dt);
        _wheel_3->control(dt);
        _wheel_4->control(dt);
    }

    void macunum_chassis_drv_t::zero_force()
    {
        _wheel_1->zero_force();
        _wheel_2->zero_force();
        _wheel_3->zero_force();
        _wheel_4->zero_force();
    }
};