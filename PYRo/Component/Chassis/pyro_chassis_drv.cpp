#include "pyro_chassis_drv.h"

#include <cmath>

#define Ox 0.17332f
#define Oy 0.16238f
#define Sy 0.17135f
#define Sx 0.1675f


namespace pyro
{

chassis_drv_t::chassis_drv_t(steering_wheel_drv_t *steering_wheel_drv_1,
                             steering_wheel_drv_t *steering_wheel_drv_2,
                             steering_wheel_drv_t *steering_wheel_drv_3,
                             steering_wheel_drv_t *steering_wheel_drv_4,
                             wheel_drv_t *wheel_drv_1,
                             wheel_drv_t *wheel_drv_2,
                             wheel_drv_t *wheel_drv_3,
                             wheel_drv_t *wheel_drv_4,
                             rc_drv_t *rc_drv)
    : _steering_wheel_drv_1(steering_wheel_drv_1),
      _steering_wheel_drv_2(steering_wheel_drv_2),
      _steering_wheel_drv_3(steering_wheel_drv_3),
      _steering_wheel_drv_4(steering_wheel_drv_4),
      _wheel_drv_1(wheel_drv_1),
      _wheel_drv_2(wheel_drv_2),
      _wheel_drv_3(wheel_drv_3),
      _wheel_drv_4(wheel_drv_4)
{
    rc_drv->config_rc_cmd([this](rc_drv_t *rc_drv) -> void { get_mode(rc_drv); });
}

void chassis_drv_t::get_mode(rc_drv_t *rc_drv)
{
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            rc_drv->get_p_ctrl());
    _vy = static_cast<float>(p_ctrl->rc.ch[3]) / 660.0f * 2.0f;
    _vx = static_cast<float>(p_ctrl->rc.ch[2]) / 660.0f * 2.0f;
    _wz = static_cast<float>(p_ctrl->rc.ch[0]) / 660.0f;
    _s_right = static_cast<uint8_t>(p_ctrl->rc.s[0]);
}

void chassis_drv_t::update_feedback()
{
    _steering_wheel_drv_1->update_feedback();
    _steering_wheel_drv_2->update_feedback();
    _steering_wheel_drv_3->update_feedback();
    _steering_wheel_drv_4->update_feedback();
    _wheel_drv_1->update_feedback();
    _wheel_drv_2->update_feedback();
    _wheel_drv_3->update_feedback();
    _wheel_drv_4->update_feedback();
}

void chassis_drv_t::zero_force()
{
    _wheel_drv_1->zero_force();
    _wheel_drv_2->zero_force();
    _steering_wheel_drv_1->zero_force();
    _steering_wheel_drv_2->zero_force();
}

void chassis_drv_t::chassis_control()
{
    if( _s_right == 1 )
    {
        zero_force();
    }
    else
    {
        if ( abs(_vx) < 0.3f && abs(_vy) > 0.3f && abs(_wz) > 0.3f )
        {
            _vx = _vy = _wz = 0;
        }

        _steering_wheel_drv_1->set_radian(atan2f(_vx + _wz * Sx, _vy + _wz * Sy));
        _steering_wheel_drv_2->set_radian(atan2f(_vx - _wz * Sx, _vy + _wz * Sy));
        _steering_wheel_drv_3->set_radian(atan2f(_vx - _wz * Sx, _vy - _wz * Sy));
        _steering_wheel_drv_4->set_radian(atan2f(_vx + _wz * Sx, _vy - _wz * Sy));


        float steering_wheel_1_speed = hypotf(_vx - _wz * Sx, _vy - _wz * Sy);
        float steering_wheel_2_speed = hypotf(_vx - _wz * Sx, _vy + _wz * Sy);
        float steering_wheel_3_speed = -hypotf(_vx + _wz * Sx, _vy + _wz * Sy);
        float steering_wheel_4_speed = -hypotf(_vx + _wz * Sx, _vy - _wz * Sy);

        _steering_wheel_drv_1->wheel_drv->set_speed(_steering_wheel_drv_1->direction * steering_wheel_1_speed);
        _steering_wheel_drv_2->wheel_drv->set_speed(_steering_wheel_drv_2->direction * steering_wheel_2_speed);
        _steering_wheel_drv_3->wheel_drv->set_speed(_steering_wheel_drv_3->direction * steering_wheel_3_speed);
        _steering_wheel_drv_4->wheel_drv->set_speed(_steering_wheel_drv_4->direction * steering_wheel_4_speed);
    }
}


}
