#include "pyro_chassis_drv.h"
#include "pyro_rc_hub.h"

#include <cmath>

#define Sy 0.17135f
#define Sx 0.1675f

float vx;
float vy;

float raw_torque;

namespace pyro
{

chassis_drv_t::chassis_drv_t(steering_wheel_drv_t *drv1,
                             steering_wheel_drv_t *drv2,
                             steering_wheel_drv_t *drv3,
                             steering_wheel_drv_t *drv4,
                             pid_t* yaw_pid,
                             uint8_t powercontrol_num
                            )
    : _steering_wheel_drv{drv1, drv2, drv3, drv4},
      _yaw_pid(yaw_pid),
      _powercontrol_num(powercontrol_num),
      _motor_data(powercontrol_num)

{
    rc_hub_t::get_instance(
        rc_hub_t::DR16)->config_rc_cmd([this](void const *rc_ctrl)->void{
            return dr16_cmd(rc_ctrl);
        });
    rc_hub_t::get_instance(
        rc_hub_t::VT03)->config_rc_cmd([this](void const *rc_ctrl)->void{
            return vt03_cmd(rc_ctrl);
        });
    power_control_drv_t::get_instance(_powercontrol_num);
}

void chassis_drv_t::dr16_cmd(void const *rc_ctrl)
{
    static auto *p_ctrl =
        static_cast<dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
    _vy      = static_cast<float>(p_ctrl->rc.ch[dr16_drv_t::DR16_CH_LEFT_Y]) * 3.8f;
    _vx      = static_cast<float>(p_ctrl->rc.ch[dr16_drv_t::DR16_CH_LEFT_X]) * 3.8f;
    _wz      = static_cast<float>(p_ctrl->rc.ch[dr16_drv_t::DR16_CH_RIGHT_X]);
    _s_right = p_ctrl->rc.s[dr16_drv_t::DR16_SW_RIGHT].state;
}

void chassis_drv_t::vt03_cmd(void const *rc_ctrl)
{

}

void chassis_drv_t::update_feedback()
{
    for(int i = 0; i < 4; i++)
    {
        _steering_wheel_drv.at(i)->update_feedback();
    }
}

void chassis_drv_t::zero_force()
{
    for(int i = 0; i < 4; i++)
    {
        _steering_wheel_drv.at(i)->zero_force();
    }
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
        if ( abs(yaw_err) < 0.008f)
        {
            yaw_err = 0;
        }

        if ( _s_right == 2 )
        {
            chassis_wz = 1.0f;
            chassis_vx = _vx * cosf(-yaw_err) - _vy * sinf(-yaw_err);
            chassis_vy = _vy * cosf(-yaw_err) + _vx * sinf(-yaw_err);
        }
        else
        {
            chassis_wz = _yaw_pid->calculate(0, yaw_err);
            chassis_vx = _vx;
            chassis_vy = _vy;
        }



        _steering_wheel_drv.at(0)->set_radian(atan2f(chassis_vx + chassis_wz * Sx, chassis_vy + chassis_wz * Sy));
        _steering_wheel_drv.at(1)->set_radian(atan2f(chassis_vx - chassis_wz * Sx, chassis_vy + chassis_wz * Sy));
        _steering_wheel_drv.at(2)->set_radian(atan2f(chassis_vx - chassis_wz * Sx, chassis_vy - chassis_wz * Sy));
        _steering_wheel_drv.at(3)->set_radian(atan2f(chassis_vx + chassis_wz * Sx, chassis_vy - chassis_wz * Sy));

        float steering_wheel_1_speed =  hypotf(chassis_vx - chassis_wz * Sx, chassis_vy - chassis_wz * Sy);
        float steering_wheel_2_speed =  hypotf(chassis_vx - chassis_wz * Sx, chassis_vy + chassis_wz * Sy);
        float steering_wheel_3_speed = -hypotf(chassis_vx + chassis_wz * Sx, chassis_vy + chassis_wz * Sy);
        float steering_wheel_4_speed = -hypotf(chassis_vx + chassis_wz * Sx, chassis_vy - chassis_wz * Sy);

        _steering_wheel_drv.at(0)->wheel_drv->set_speed(_steering_wheel_drv.at(0)->direction * steering_wheel_1_speed);
        _steering_wheel_drv.at(1)->wheel_drv->set_speed(_steering_wheel_drv.at(1)->direction * steering_wheel_2_speed);
        _steering_wheel_drv.at(2)->wheel_drv->set_speed(_steering_wheel_drv.at(2)->direction * steering_wheel_3_speed);
        _steering_wheel_drv.at(3)->wheel_drv->set_speed(_steering_wheel_drv.at(3)->direction * steering_wheel_4_speed);

        #if POWER_CONTROL_USE

        power_control_drv_t& power_controller = power_control_drv_t::get_instance();
        for(int i = 0; i < 4; i++)
        {
            _motor_data.at(i).gyro = _steering_wheel_drv.at(i)->wheel_drv->rotate;
            _motor_data.at(i).torque_cmd = _steering_wheel_drv.at(i)->wheel_drv->torque_cmd;
            _motor_data.at(i).power_predict = power_controller.motor_power_predict(i, 
                    _motor_data.at(i).torque_cmd, _motor_data.at(i).gyro);
        }

        // float custom_ratios[4] = {0.1f, 0.1f, 0.1f, 0.1f};
        // power_controller.calculate_restricted_torques(_wheel_data.data(), 4, POWER_LIMIT, custom_ratios);
        power_controller.calculate_restricted_torques(_motor_data.data(), 4, POWER_LIMIT);
        for(int i = 0; i < 4; i++)
        {
            _steering_wheel_drv.at(i)->wheel_drv->torque_cmd = _motor_data.at(i).restricted_torque;
        }

        #endif

        _steering_wheel_drv.at(0)->wheel_drv->control();
        _steering_wheel_drv.at(1)->wheel_drv->control();
        _steering_wheel_drv.at(2)->wheel_drv->control();
        _steering_wheel_drv.at(3)->wheel_drv->control();
    }


}


} // namespace pyro
