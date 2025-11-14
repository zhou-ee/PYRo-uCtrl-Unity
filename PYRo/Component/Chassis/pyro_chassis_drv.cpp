#include "pyro_chassis_drv.h"
#include "pyro_rc_hub.h"

#include <cmath>

#define Ox 0.17332f
#define Oy 0.16238f
#define Sy 0.17135f
#define Sx 0.1675f

float vx;
float vy;
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
                             pid_ctrl_t* yaw_pid)
    : _steering_wheel_drv_1(steering_wheel_drv_1),
      _steering_wheel_drv_2(steering_wheel_drv_2),
      _steering_wheel_drv_3(steering_wheel_drv_3),
      _steering_wheel_drv_4(steering_wheel_drv_4),
      _wheel_drv_1(wheel_drv_1),
      _wheel_drv_2(wheel_drv_2),
      _wheel_drv_3(wheel_drv_3),
      _wheel_drv_4(wheel_drv_4),
      _yaw_pid(yaw_pid)
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

void chassis_drv_t::dr16_cmd(void const *rc_ctrl)
{
    static auto *p_ctrl =
        static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
    _vy      = static_cast<float>(p_ctrl->rc.ch[dr16_drv_t::DR16_CH_LEFT_Y]) * 2.0f;
    _vx      = static_cast<float>(p_ctrl->rc.ch[dr16_drv_t::DR16_CH_LEFT_X]) * 2.0f;
    _wz      = static_cast<float>(p_ctrl->rc.ch[dr16_drv_t::DR16_CH_RIGHT_X]);
    _s_right = p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT].state;
}

void chassis_drv_t::vt03_cmd(void const *rc_ctrl)
{

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
    _wheel_drv_3->zero_force();
    _wheel_drv_4->zero_force();
    _steering_wheel_drv_1->zero_force();
    _steering_wheel_drv_2->zero_force();
    _steering_wheel_drv_3->zero_force();
    _steering_wheel_drv_4->zero_force();
}

void chassis_drv_t::chassis_control(float yaw_err)
{
    float chassis_vx;
    float chassis_vy;
    float chassis_wz;


    if( _s_right == 1 )
    {
        chassis_vx = chassis_vy = chassis_wz = 0;
        zero_force();
    }
    else
    {
        if ( abs(_vx) < 0.3f && abs(_vy) < 0.3f && abs(_wz) < 0.3f )
        {
            _vx = _vy = _wz = 0;
        }
        if ( abs(yaw_err) < 0.05f)
        {
            yaw_err = 0;
        }

        if ( _s_right == 2 )
        {
            chassis_wz = 0.5f;
            chassis_vx = _vx * cosf(-yaw_err) - _vy * sinf(-yaw_err);
            chassis_vy = _vy * cosf(-yaw_err) + _vx * sinf(-yaw_err);
        }
        else
        {
            chassis_wz = _yaw_pid->compute(0, yaw_err, 0.001);
            chassis_vx = _vx;
            chassis_vy = _vy;
        }



        _steering_wheel_drv_1->set_radian(atan2f(chassis_vx + chassis_wz * Sx, chassis_vy + chassis_wz * Sy));
        _steering_wheel_drv_2->set_radian(atan2f(chassis_vx - chassis_wz * Sx, chassis_vy + chassis_wz * Sy));
        _steering_wheel_drv_3->set_radian(atan2f(chassis_vx - chassis_wz * Sx, chassis_vy - chassis_wz * Sy));
        _steering_wheel_drv_4->set_radian(atan2f(chassis_vx + chassis_wz * Sx, chassis_vy - chassis_wz * Sy));


        float steering_wheel_1_speed = hypotf(chassis_vx - chassis_wz * Sx, chassis_vy - chassis_wz * Sy);
        float steering_wheel_2_speed = hypotf(chassis_vx - chassis_wz * Sx, chassis_vy + chassis_wz * Sy);
        float steering_wheel_3_speed = -hypotf(chassis_vx + chassis_wz * Sx, chassis_vy + chassis_wz * Sy);
        float steering_wheel_4_speed = -hypotf(chassis_vx + chassis_wz * Sx, chassis_vy - chassis_wz * Sy);

        _steering_wheel_drv_1->wheel_drv->set_speed(_steering_wheel_drv_1->direction * steering_wheel_1_speed);
        _steering_wheel_drv_2->wheel_drv->set_speed(_steering_wheel_drv_2->direction * steering_wheel_2_speed);
        _steering_wheel_drv_3->wheel_drv->set_speed(_steering_wheel_drv_3->direction * steering_wheel_3_speed);
        _steering_wheel_drv_4->wheel_drv->set_speed(_steering_wheel_drv_4->direction * steering_wheel_4_speed);
    }


}


} // namespace pyro
