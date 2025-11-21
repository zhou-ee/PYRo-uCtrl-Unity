#include "pyro_rud_chassis.h"
#include "pyro_dji_motor_drv.h"

namespace pyro
{

rud_chassis_t::~rud_chassis_t()
{
    for (int i = 0; i < 4; ++i)
    {
        delete _wheel_motor[i];
        delete _rudder_motor[i];
        delete _wheel_speed_pid[i];
        delete _rudder_angle_pid[i];
        delete _rudder_speed_pid[i];
    }
    delete _follow_angle_pid;
}


void rud_chassis_t::init()
{
    _kinematics = new rudder_kin_t(
        0.35f, 0.35f); // Example dimensions: 0.35m wheelbase, 0.35m track width

    _wheel_motor[0]  = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                                 can_hub_t::can1); // FL
    _wheel_motor[1]  = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                 can_hub_t::can1); // FR
    _wheel_motor[2]  = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                                 can_hub_t::can1); // BL
    _wheel_motor[3]  = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                                 can_hub_t::can1); // BR

    _rudder_motor[0] = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                                   can_hub_t::can2);
    _rudder_motor[1] = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                   can_hub_t::can2);
    _rudder_motor[2] = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                                   can_hub_t::can2);
    _rudder_motor[3] = new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                                   can_hub_t::can2);

    _wheel_speed_pid[0]  = new pid_t(18.0f, 0.0f, 0.0f, 1.0f, 20.0f); // FL
    _wheel_speed_pid[1]  = new pid_t(18.0f, 0.0f, 0.0f, 1.0f, 20.0f); // FR
    _wheel_speed_pid[2]  = new pid_t(18.0f, 0.0f, 0.0f, 1.0f, 20.0f); // BL
    _wheel_speed_pid[3]  = new pid_t(18.0f, 0.0f, 0.0f, 1.0f, 20.0f); // BR

    _rudder_angle_pid[0] = new pid_t(18.0f, 0.0f, 0.0f, 0.5f, 10.0f);
    _rudder_angle_pid[1] = new pid_t(18.0f, 0.0f, 0.0f, 0.5f, 10.0f);
    _rudder_angle_pid[2] = new pid_t(18.0f, 0.0f, 0.0f, 0.5f, 10.0f);
    _rudder_angle_pid[3] = new pid_t(18.0f, 0.0f, 0.0f, 0.5f, 10.0f);
    _rudder_speed_pid[0] = new pid_t(8.0f, 0.0f, 0.0f, 0.5f, 3.0f);
    _rudder_speed_pid[1] = new pid_t(8.0f, 0.0f, 0.0f, 0.5f, 3.0f);
    _rudder_speed_pid[2] = new pid_t(8.0f, 0.0f, 0.0f, 0.5f, 3.0f);
    _rudder_speed_pid[3] = new pid_t(8.0f, 0.0f, 0.0f, 0.5f, 3.0f);

    _follow_angle_pid    = new pid_t(10.0f, 0.0f, 0.0f, 1.0f, 5.0f);
}

void rud_chassis_t::set_command(const cmd_base_t &cmd)
{
    _cmd_rud = (const cmd_rud_t &)cmd;
}

void rud_chassis_t::update_feedback()
{
    for (int i = 0; i < 4; ++i)
    {
        _wheel_motor[i]->update_feedback();
        _rudder_motor[i]->update_feedback();
    }
    _current_states = {
        {
            _wheel_motor[0]->get_current_rotate() *
                dji_m3508_motor_drv_t::reciprocal_reduction_ratio,
            _rudder_motor[0]->get_current_position(),
        },
        {
            _wheel_motor[1]->get_current_rotate() *
                dji_m3508_motor_drv_t::reciprocal_reduction_ratio,
            _rudder_motor[1]->get_current_position(),
        },
        {
            _wheel_motor[2]->get_current_rotate() *
                dji_m3508_motor_drv_t::reciprocal_reduction_ratio,
            _rudder_motor[2]->get_current_position(),
        },
        {
            _wheel_motor[3]->get_current_rotate() *
                dji_m3508_motor_drv_t::reciprocal_reduction_ratio,
            _rudder_motor[3]->get_current_position(),
        },
    };
    for (int i = 0; i < 4; ++i)
    {
        _rudder_current_speed[i] = _rudder_motor[i]->get_current_rotate();
    }
}

void rud_chassis_t::kinematics_solve()
{
    if (0 == _cmd_rud.wz)
    {
        _cmd_rud.wz += _follow_angle_pid->calculate(0, _cmd_rud.yaw_err);
    }
    _target_states = _kinematics->solve(_cmd_rud.vx, _cmd_rud.vy, _cmd_rud.wz,
                                        _current_states);
}


void rud_chassis_t::chassis_control()
{
    _wheel_output[0] = _wheel_speed_pid[0]->calculate(_target_states.fl.speed,
                                                      _current_states.fl.speed);
    _wheel_output[1] = _wheel_speed_pid[1]->calculate(_target_states.fr.speed,
                                                      _current_states.fr.speed);
    _wheel_output[2] = _wheel_speed_pid[2]->calculate(_target_states.fl.speed,
                                                      _current_states.bl.speed);
    _wheel_output[3] = _wheel_speed_pid[3]->calculate(_target_states.br.speed,
                                                      _current_states.br.speed);

    _rudder_target_speed[0] = _rudder_angle_pid[0]->calculate(
        _target_states.fl.angle, _current_states.fl.angle);
    _rudder_target_speed[1] = _rudder_angle_pid[1]->calculate(
        _target_states.fr.angle, _current_states.fr.angle);
    _rudder_target_speed[2] = _rudder_angle_pid[2]->calculate(
        _target_states.bl.angle, _current_states.bl.angle);
    _rudder_target_speed[3] = _rudder_angle_pid[3]->calculate(
        _target_states.br.angle, _current_states.br.angle);

    _rudder_output[0] = _rudder_speed_pid[0]->calculate(
        _rudder_target_speed[0], _rudder_current_speed[0]);
    _rudder_output[1] = _rudder_speed_pid[1]->calculate(
        _rudder_target_speed[1], _rudder_current_speed[1]);
    _rudder_output[2] = _rudder_speed_pid[2]->calculate(
        _rudder_target_speed[2], _rudder_current_speed[2]);
    _rudder_output[3] = _rudder_speed_pid[3]->calculate(
        _rudder_target_speed[3], _rudder_current_speed[3]);
}

void rud_chassis_t::power_control()
{

}

void rud_chassis_t::send_motor_command()
{
    for (int i = 0; i < 4; ++i)
    {
        _wheel_motor[i]->send_torque(_wheel_output[i]);
        _rudder_motor[i]->send_torque(_rudder_output[i]);
    }
}


} // namespace pyro