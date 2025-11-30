#include "pyro_hybrid_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"


namespace pyro
{

hybrid_chassis_t::hybrid_chassis_t() : chassis_base_t(type_t::HYBRID)
{
}

hybrid_chassis_t::~hybrid_chassis_t()
{
    delete _kinematics;
    delete _balance_pid;

    for (int i = 0; i < 4; ++i)
    {
        delete _mecanum_motor[i];
        delete _mecanum_speed_pid[i];
    }
    for (int i = 0; i < 2; ++i)
    {
        delete _track_motor[i];
        delete _track_speed_pid[i];
        delete _leg_motor[i];
        delete _leg_position_pid[i];
    }
}

void hybrid_chassis_t::init()
{
    // 1. Initialize Kinematics
    // Params: Track Spacing (m), Mecanum Wheelbase (m), Mecanum Track Width (m)
    // Replace 0.4f, 0.3f, 0.35f with your actual robot dimensions
    _kinematics       = new hybrid_kin_t(0.648f, 0.35f, 0.41f);

    // 2. Initialize Motors
    // Mecanum (DJI M3508)
    _mecanum_motor[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                                  can_hub_t::can1); // FL
    _mecanum_motor[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                  can_hub_t::can1); // FR
    _mecanum_motor[2] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                                  can_hub_t::can1); // BL
    _mecanum_motor[3] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                                  can_hub_t::can1); // BR

    // NOLINTBEGIN(cppcoreguidelines-pro-type-static-cast-downcast)

    // Tracks (DM Motors) - Assuming control mode is set internally or defaults
    // to MIT/Velocity Note: You might need to set the control mode for DM
    // motors (Speed vs Position)
    _track_motor[0]   = new dm_motor_drv_t(0x11, 0x21, can_hub_t::can2);
    _track_motor[1]   = new dm_motor_drv_t(0x12, 0x22, can_hub_t::can2);
    static_cast<dm_motor_drv_t *>(_track_motor[0])->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(_track_motor[0])->set_rotate_range(-50, 50);
    static_cast<dm_motor_drv_t *>(_track_motor[0])->set_torque_range(-10, 10);
    static_cast<dm_motor_drv_t *>(_track_motor[1])->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(_track_motor[1])->set_rotate_range(-50, 50);
    static_cast<dm_motor_drv_t *>(_track_motor[1])->set_torque_range(-10, 10);


    // Legs (DM Motors)
    _leg_motor[0] = new dm_motor_drv_t(0x31, 0x41, can_hub_t::can3);
    _leg_motor[1] = new dm_motor_drv_t(0x32, 0x42, can_hub_t::can3);
    static_cast<dm_motor_drv_t *>(_leg_motor[0])->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(_leg_motor[0])->set_rotate_range(-10, 10);
    static_cast<dm_motor_drv_t *>(_leg_motor[0])->set_torque_range(-10, 10);
    static_cast<dm_motor_drv_t *>(_leg_motor[1])->set_position_range(-PI, PI);
    static_cast<dm_motor_drv_t *>(_leg_motor[1])->set_rotate_range(-10, 10);
    static_cast<dm_motor_drv_t *>(_leg_motor[1])->set_torque_range(-10, 10);

    // NOLINTEND(cppcoreguidelines-pro-type-static-cast-downcast)

    // 3. Initialize PIDs
    // PID Params: MaxOut, IntergralLimit, Kp, Ki, Kd

    // Mecanum Speed Loop
    _mecanum_speed_pid[0] =
        new pid_t(0.4f, 0.001f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    _mecanum_speed_pid[1] =
        new pid_t(0.4f, 0.001f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    _mecanum_speed_pid[2] =
        new pid_t(0.4f, 0.001f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    _mecanum_speed_pid[3] =
        new pid_t(0.4f, 0.001f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);

    // Track Speed Loop
    for (auto &i : _track_speed_pid)
    {
        i = new pid_t(0.045f, 0.0001f, 0.000005f, 0.5f, 10.0f, 30, 10, 4);
    }

    // Leg Position Loop (Inner loop)
    for (auto &i : _leg_position_pid)
    {
        i = new pid_t(8.0f, 0.005f, 0.0012f, 0.5f, 10.0f, 20, 10, 4);
    }

    for (auto &i : _leg_rotate_pid)
    {
        i = new pid_t(8.0f, 0.005f, 0.0012f, 0.5f, 10.0f, 20, 10, 4);
    }

    // Balance PID (Outer loop: Pitch -> Leg Position)
    // Positive Output implies extending legs to push rear up
    _balance_pid = new pid_t(3.14f, 0.5f, 0.1f, 0.0f,
                             0.05f); // Output is radians (position offset)
}

void hybrid_chassis_t::update_imu_data(const float pitch_rad,
                                       const float roll_rad,
                                       const float yaw_rad)
{
    _current_pitch_rad = pitch_rad;
    _current_roll_rad  = roll_rad;
    _current_yaw_rad   = yaw_rad;
}

void hybrid_chassis_t::set_command(const cmd_base_t *cmd)
{
    // Safe cast to hybrid command to access specific modes
    if (cmd->type == type_t::HYBRID)
    {
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-static-cast-downcast)
        _cmd_hybrid = static_cast<const cmd_hybrid_t *>(cmd);
    }
}

void hybrid_chassis_t::update_feedback()
{
    for (const auto &i : _mecanum_motor)
        i->update_feedback();
    for (const auto &i : _track_motor)
        i->update_feedback();
    for (const auto &i : _leg_motor)
        i->update_feedback();

    // 1. 四个轮子的 RPM
    _current_wheel_rpm[0] =
        _radps_to_rpm(_mecanum_motor[0]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    _current_wheel_rpm[1] =
        _radps_to_rpm(_mecanum_motor[1]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    _current_wheel_rpm[2] =
        _radps_to_rpm(_mecanum_motor[2]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    _current_wheel_rpm[3] =
        _radps_to_rpm(_mecanum_motor[3]->get_current_rotate() *
                      dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

    // 2. 两条履带的 RPM、腿角度、腿角速度
    _current_track_rpm[0] =
        _radps_to_rpm(_track_motor[0]->get_current_rotate());
    _current_track_rpm[1] =
        _radps_to_rpm(_track_motor[1]->get_current_rotate());

    _current_leg_rad[0]   = _leg_motor[0]->get_current_position();
    _current_leg_radps[0] = _leg_motor[0]->get_current_rotate();
    _current_leg_rad[1]   = _leg_motor[1]->get_current_position();
    _current_leg_radps[1] = _leg_motor[1]->get_current_rotate();

    update_imu_data(0, 0, 0);
}

void hybrid_chassis_t::kinematics_solve()
{
    if (mode_t::ZERO_FORCE == _cmd_hybrid->mode)
    {
        return;
    }
    // 1. Solve for Traction Motors (Mecanum + Tracks)
    _solved_speeds_mps =
        _kinematics->solve(_cmd_hybrid->vx, _cmd_hybrid->vy, _cmd_hybrid->wz,
                           _cmd_hybrid->drive_mode);

    _target_wheel_rpm[0] =
        _mps_to_rpm(_solved_speeds_mps.mec_fl, MEC_WHEEL_RADIUS_M);
    _target_wheel_rpm[1] =
        -_mps_to_rpm(_solved_speeds_mps.mec_fr, MEC_WHEEL_RADIUS_M);
    _target_wheel_rpm[2] =
        _mps_to_rpm(_solved_speeds_mps.mec_bl, MEC_WHEEL_RADIUS_M);
    _target_wheel_rpm[3] =
        -_mps_to_rpm(_solved_speeds_mps.mec_br, MEC_WHEEL_RADIUS_M);

    _target_track_rpm[0] =
        0.75f * _mps_to_rpm(_solved_speeds_mps.track_l, TRACK_WHEEL_RADIUS_M);
    _target_track_rpm[1] =
        -0.75f * _mps_to_rpm(_solved_speeds_mps.track_r, TRACK_WHEEL_RADIUS_M);
    static uint8_t jump_flag = 0;

    if (0 == _cmd_hybrid->leg_contract_mode)
    {
        jump_flag = 0;
        _leg_position_pid[0]->clear();
        _leg_position_pid[1]->clear();
        _leg_rotate_pid[0]->clear();
        _leg_rotate_pid[1]->clear();
        _leg_position_pid[0]->set_gains(8.0f, 0.005f, 0.0012f);
        _leg_position_pid[1]->set_gains(8.0f, 0.005f, 0.0012f);
        _leg_rotate_pid[0]->set_gains(8.0f, 0.005f, 0.0012f);
        _leg_rotate_pid[1]->set_gains(8.0f, 0.005f, 0.0012f);
        _target_leg_rad[0] += _cmd_hybrid->wy;
        _target_leg_rad[1] += -_cmd_hybrid->wy;
        if (_target_leg_rad[0] > LEG_EXTEND_POS)
            _target_leg_rad[0] = LEG_EXTEND_POS;
        if (_target_leg_rad[0] < 0.2f)
            _target_leg_rad[0] = 0.2f;
        if (_target_leg_rad[1] < -LEG_EXTEND_POS)
            _target_leg_rad[1] = -LEG_EXTEND_POS;
        if (_target_leg_rad[1] > -0.2f)
            _target_leg_rad[1] = -0.2f;
    }
    else
    {
        _leg_position_pid[0]->clear();
        _leg_position_pid[1]->clear();
        _leg_rotate_pid[0]->clear();
        _leg_rotate_pid[1]->clear();
        _leg_position_pid[0]->set_gains(40.0f, 0.01f, 0.002f);
        _leg_position_pid[1]->set_gains(40.0f, 0.01f, 0.002f);
        _leg_rotate_pid[0]->set_gains(100.0f, 0.01f, 0.002f);
        _leg_rotate_pid[1]->set_gains(500.0f, 0.01f, 0.002f);
        if (jump_flag == 0)
        {
            _target_leg_rad[0] = 0.2f;
            _target_leg_rad[1] = -0.2f;
            if (abs(_current_leg_rad[0] - _target_leg_rad[0]) < 0.2f &&
                abs(_current_leg_rad[1] - _target_leg_rad[1]) < 0.2f)
            {
                jump_flag = 1;
            }
        }

        if (jump_flag == 1)
        {
            jump_flag          = 2;
            _target_leg_rad[0] = LEG_EXTEND_POS;
            _target_leg_rad[1] = -LEG_EXTEND_POS;
        }
        if (jump_flag == 2)
        {
            if (abs(_current_leg_rad[0] - _target_leg_rad[0]) < 0.2f &&
                abs(_current_leg_rad[1] - _target_leg_rad[1]) < 0.2f)
            {
                jump_flag          = 3;
                _target_leg_rad[0] = 0.2f;
                _target_leg_rad[1] = -0.2f;
            }
        }
    }
}

void hybrid_chassis_t::chassis_control()
{
    if (mode_t::ZERO_FORCE == _cmd_hybrid->mode)
    {
        return;
    }

    for (int i = 0; i < 4; ++i)
    {
        _mecanum_output_torque[i] = _mecanum_speed_pid[i]->calculate(
            _target_wheel_rpm[i], _current_wheel_rpm[i]);
    }

    // Leg Position Loop
    for (int i = 0; i < 2; ++i)
    {
        // _target_leg_rad[i] =
        //     _balance_pid->calculate(0.0f, _current_pitch_rad); // Pitch to
        //     Rad
        _target_leg_radps[i] = _leg_position_pid[i]->calculate(
            _target_leg_rad[i], _current_leg_rad[i]);
        _leg_output_torque[i] = _leg_rotate_pid[i]->calculate(
            _target_leg_radps[i], _current_leg_radps[i]);
    }

    if (hybrid_kin_t::drive_mode_t::CLIMBING == _cmd_hybrid->drive_mode)
    {
        for (int i = 0; i < 2; ++i)
        {
            _track_output_torque[i] = _track_speed_pid[i]->calculate(
                _target_track_rpm[i], _current_track_rpm[i]);
        }
    }
    else
    {
        // In cruising mode, tracks are not used
        _track_output_torque[0] = 0.0f;
        _track_output_torque[1] = 0.0f;
    }
}

void hybrid_chassis_t::power_control()
{
    // Implement supercapacitor or power limit logic here if needed
}

void hybrid_chassis_t::send_motor_command()
{
    for (const auto &i : _track_motor)
    {
        if (!i->is_enable())
        {
            i->enable();
        }
    }
    for (const auto &i : _leg_motor)
    {
        if (!i->is_enable())
        {
            i->enable();
        }
    }
    // _track_motor[0]->send_torque(_track_output_torque[0]);//正
    // _track_motor[1]->send_torque(_track_output_torque[1]);//反

    // _leg_motor[0]->send_torque(0.0f);   // 往上是正
    // _leg_motor[1]->send_torque(0.0f);   // 往上是负
    // _track_motor[0]->send_torque(0.0f); // 正
    // _track_motor[1]->send_torque(0.0f); // 反

    if (mode_t::ZERO_FORCE == _cmd_hybrid->mode)
    {
        for (const auto &i : _mecanum_motor)
            i->send_torque(0.0f);
        for (const auto &i : _track_motor)
            i->send_torque(0.0f);
        for (const auto &i : _leg_motor)
            i->send_torque(0.0f);
        return;
    }
    // _leg_motor[0]->send_torque(10.0f);   // 往上是正
    // _leg_motor[1]->send_torque(-10.0f);  // 往上是负
    //
    for (int i = 0; i < 4; ++i)
        _mecanum_motor[i]->send_torque(_mecanum_output_torque[i]);
    for (int i = 0; i < 2; ++i)
    {
        _track_motor[i]->send_torque(_track_output_torque[i]);
    }

    for (int i = 0; i < 2; ++i)
    {
        _leg_motor[i]->send_torque(_leg_output_torque[i]);
    }
}

float hybrid_chassis_t::_mps_to_rpm(const float mps, const float radius)
{
    // v = w * r  -> w = v / r
    // RPM = w * 60 / 2pi
    if (radius < 1e-4f)
        return 0.0f;
    return (mps / radius) * 9.5492966f;
}

float hybrid_chassis_t::_radps_to_rpm(const float radps)
{
    // RPM = (w * 60) / (2 * pi)
    return radps * 9.5492966f;
}

} // namespace pyro