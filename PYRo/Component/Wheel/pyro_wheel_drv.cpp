#include "pyro_wheel_drv.h"

namespace pyro
{

pyro_wheel_drv_t::pyro_wheel_drv_t(pyro::dji_motor_tx_frame_t::register_id_t id,
                 can_hub_t::which_can which,
                 float radius,
                 const pid_ctrl_t &speed_pid,
                 rc_drv_t *rc_drv)
    : dji_m3508_motor_drv_t(id, which),
    _radius(radius),
    _speed_pid(speed_pid)
{
    rc_drv->set_get_mode([this](rc_drv_t *rc_drv) -> void { get_mode(rc_drv); });  
};

void pyro_wheel_drv_t::set_rotate(float target_rotate)
{
    float current_rotate = get_current_rotate();
    float torque_cmd = _speed_pid.compute(target_rotate, current_rotate / 19.0f, 0.001f);
    send_torque(torque_cmd);
}

void pyro_wheel_drv_t::get_mode(rc_drv_t *rc_drv)
{
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            rc_drv->get_p_ctrl());
    static auto *p_last_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            rc_drv->get_p_last_ctrl());
    _target_rotate = static_cast<float>(p_ctrl->rc.ch[0]) / 660.0f * 10.0f; 
}

float pyro_wheel_drv_t::get_target_rotate()
{
    return _target_rotate;
}

}
