#include "pyro_hybrid_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"


namespace pyro
{

hybrid_chassis_t::hybrid_chassis_t() : chassis_base_t(type_t::WHEEL_LEG)
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
    _kinematics       = new hybrid_kin_t(0.40f, 0.30f, 0.35f);

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

    // Tracks (DM Motors) - Assuming control mode is set internally or defaults
    // to MIT/Velocity Note: You might need to set the control mode for DM
    // motors (Speed vs Position)
    _track_motor[0] =
        new dm_motor_drv_t(0x11, 0x21, can_hub_t::can2); // Left Track
    _track_motor[1] = new dm_motor_drv_t(0x12, 0x22, can_hub_t::can2);

    // Legs (DM Motors)
    _leg_motor[0]   = new dm_motor_drv_t(0x31, 0x41, can_hub_t::can3);
    _leg_motor[1]   = new dm_motor_drv_t(0x32, 0x42, can_hub_t::can3);

    // 3. Initialize PIDs
    // PID Params: MaxOut, IntergralLimit, Kp, Ki, Kd

    // Mecanum Speed Loop
    for (auto &i : _mecanum_speed_pid)
    {
        i = new pid_t(16000.0f, 5000.0f, 50.0f, 0.5f, 10.0f);
    }

    // Track Speed Loop
    for (auto &i : _track_speed_pid)
    {
        i = new pid_t(10.0f, 2.0f, 1.0f, 0.0f, 0.1f);
    }

    // Leg Position Loop (Inner loop)
    for (auto &i : _leg_position_pid)
    {
        i = new pid_t(10.0f, 1.0f, 8.0f, 0.01f, 0.5f);
    }

    // Balance PID (Outer loop: Pitch -> Leg Position)
    // Positive Output implies extending legs to push rear up
    _balance_pid = new pid_t(3.14f, 0.5f, 0.1f, 0.0f,
                             0.05f); // Output is radians (position offset)
}

void hybrid_chassis_t::update_imu_data(const float pitch_deg)
{
    _current_pitch_deg = pitch_deg;
}

void hybrid_chassis_t::set_command(const cmd_base_t &cmd)
{
    // Safe cast to hybrid command to access specific modes
    if (cmd.type == type_t::WHEEL_LEG)
    {
        _cmd_hybrid = (const cmd_hybrid_t &)cmd;
    }
    else
    {
        // Fallback for generic commands, assume Cruising
        _cmd_hybrid.vx         = cmd.vx;
        _cmd_hybrid.vy         = cmd.vy;
        _cmd_hybrid.wz         = cmd.wz;
        _cmd_hybrid.timestamp  = cmd.timestamp;
        _cmd_hybrid.drive_mode = hybrid_kin_t::drive_mode_t::CRUISING;
    }
}

void hybrid_chassis_t::update_feedback()
{
    for (const auto & i : _mecanum_motor)
        i->update_feedback();
    for (const auto & i : _track_motor)
        i->update_feedback();
    for (const auto & i : _leg_motor)
        i->update_feedback();
}

void hybrid_chassis_t::kinematics_solve()
{
    // 1. Solve for Traction Motors (Mecanum + Tracks)
    _target_speeds_mps = _kinematics->solve(
        _cmd_hybrid.vx, _cmd_hybrid.vy, _cmd_hybrid.wz, _cmd_hybrid.drive_mode);
}

void hybrid_chassis_t::chassis_control()
{
    // --- 1. Traction Control (Velocity Loop) ---

    // Mecanum Control (Convert m/s to RPM or Rad/s for PID)
    // Note: Assuming motor driver feedback is in RPM. Adjust conversions as
    // needed.
    const float mec_targets[4] = {
        mps_to_rpm(_target_speeds_mps.mec_fl, MEC_WHEEL_RADIUS_M),
        mps_to_rpm(_target_speeds_mps.mec_fr, MEC_WHEEL_RADIUS_M),
        mps_to_rpm(_target_speeds_mps.mec_bl, MEC_WHEEL_RADIUS_M),
        mps_to_rpm(_target_speeds_mps.mec_br, MEC_WHEEL_RADIUS_M)};

    for (int i = 0; i < 4; ++i)
    {
        _mecanum_output[i] = _mecanum_speed_pid[i]->calculate(
            mec_targets[i],
            _mecanum_motor[i]->get_current_rotate() // Assuming this returns RPM
        );
    }

    // Track Control
    float track_targets[2] = {
        mps_to_rpm(_target_speeds_mps.track_l, TRACK_WHEEL_RADIUS_M),
        mps_to_rpm(_target_speeds_mps.track_r, TRACK_WHEEL_RADIUS_M)};

    for (int i = 0; i < 2; ++i)
    {
        _track_output[i] = _track_speed_pid[i]->calculate(
            track_targets[i], _track_motor[i]->get_current_rotate());
    }

    // --- 2. Leg Control (Balance Logic) ---
    float target_leg_pos = LEG_RETRACT_POS;

    if (_cmd_hybrid.drive_mode == hybrid_kin_t::drive_mode_t::CLIMBING)
    {
        // Active Balancing:
        // If Pitch > Target (Nose up), we need to EXTEND legs (Push rear up).
        // Error = Current - Target
        float pitch_error = _current_pitch_deg - _cmd_hybrid.target_pitch_deg;

        // Calculate offset (in Radians)
        float balance_offset =
            _balance_pid->calculate(pitch_error, 0.0f); // Setpoint is 0 error

        // Base climbing position + stabilization
        // Usually climbing needs legs partially extended initially
        float base_climb_pos = (LEG_EXTEND_POS + LEG_RETRACT_POS) * 0.5f;

        target_leg_pos       = base_climb_pos + balance_offset;

        // Constraint
        if (target_leg_pos > LEG_EXTEND_POS)
            target_leg_pos = LEG_EXTEND_POS;
        if (target_leg_pos < LEG_RETRACT_POS)
            target_leg_pos = LEG_RETRACT_POS;
    }
    else
    {
        // Cruising: Retract legs completely
        target_leg_pos = LEG_RETRACT_POS;
        _balance_pid->clear(); // Reset integral term
    }

    // Leg Position Loop
    for (int i = 0; i < 2; ++i)
    {
        _leg_output[i] = _leg_position_pid[i]->calculate(
            target_leg_pos,
            _leg_motor[i]->get_current_position() // Assuming returns Rad
        );
    }
}

void hybrid_chassis_t::power_control()
{
    // Implement supercapacitor or power limit logic here if needed
}

void hybrid_chassis_t::send_motor_command()
{
    for (int i = 0; i < 4; ++i)
        _mecanum_motor[i]->send_torque(_mecanum_output[i]);
    for (int i = 0; i < 2; ++i)
        _track_motor[i]->send_torque(_track_output[i]);
    for (int i = 0; i < 2; ++i)
        _leg_motor[i]->send_torque(_leg_output[i]);
}

float hybrid_chassis_t::mps_to_rpm(float mps, float radius)
{
    // v = w * r  -> w = v / r
    // RPM = w * 60 / 2pi
    if (radius < 1e-4f)
        return 0.0f;
    return (mps / radius) * 9.5492966f;
}

} // namespace pyro