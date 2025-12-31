#include "pyro_hybrid_chassis.h"
#include <cstring>
#include <cmath>

namespace pyro
{

// =========================================================
// 构造与析构
// =========================================================

// 私有构造函数，由 chassis_base_t::instance() 调用
hybrid_chassis_t::hybrid_chassis_t()
    // 传递给基类的参数：名字, init栈, loop栈, 优先级
    : chassis_base_t("hybrid", 512, 512, task_base_t::priority_t::HIGH)
{
    std::memset(_target_wheel_rpm, 0, sizeof(_target_wheel_rpm));
    std::memset(_target_track_rpm, 0, sizeof(_target_track_rpm));
    std::memset(_target_leg_rad, 0, sizeof(_target_leg_rad));
}

hybrid_chassis_t::~hybrid_chassis_t()
{
    delete _kinematics;
    delete _balance_pid;
    delete _power_meter;
    delete _power_data;

    for (int i = 0; i < 4; ++i) {
        delete _mecanum_motor[i];
        delete _mecanum_speed_pid[i];
    }
    for (int i = 0; i < 2; ++i) {
        delete _track_motor[i];
        delete _track_speed_pid[i];
        delete _leg_motor[i];
        delete _leg_position_pid[i];
        delete _leg_rotate_pid[i];
    }
}

// =========================================================
// 初始化逻辑 (_init 代理)
// =========================================================
void hybrid_chassis_t::_init()
{
    // 1. 初始化运动学
    _kinematics = new hybrid_kin_t(0.648f, 0.35f, 0.41f);

    // 2. 初始化麦轮电机 (DJI 3508)
    _mecanum_motor[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can1);
    _mecanum_motor[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, can_hub_t::can1);
    _mecanum_motor[2] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, can_hub_t::can1);
    _mecanum_motor[3] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, can_hub_t::can1);

    // 3. 初始化履带电机 (DM Motor)
    _track_motor[0] = new dm_motor_drv_t(0x11, 0x21, can_hub_t::can2);
    _track_motor[1] = new dm_motor_drv_t(0x12, 0x22, can_hub_t::can2);

    auto conf_dm = [](motor_base_t* m, float t_range, float v_range) {
        auto* dm = static_cast<dm_motor_drv_t*>(m);
        dm->set_position_range(-PI, PI);
        dm->set_rotate_range(-v_range, v_range);
        dm->set_torque_range(-t_range, t_range);
    };
    conf_dm(_track_motor[0], 10.0f, 50.0f);
    conf_dm(_track_motor[1], 10.0f, 50.0f);

    // 4. 初始化腿部电机 (DM Motor)
    _leg_motor[0] = new dm_motor_drv_t(0x31, 0x41, can_hub_t::can3);
    _leg_motor[1] = new dm_motor_drv_t(0x32, 0x42, can_hub_t::can3);
    conf_dm(_leg_motor[0], 10.0f, 10.0f);
    conf_dm(_leg_motor[1], 10.0f, 10.0f);

    // 5. PID 初始化
    // Mecanum
    _mecanum_speed_pid[0] = new pid_t(0.28f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    _mecanum_speed_pid[1] = new pid_t(0.35f, 0.0007f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    _mecanum_speed_pid[2] = new pid_t(0.37f, 0.0008f, 0.0002f, 1.0f, 20.0f, 20, 10, 4);
    _mecanum_speed_pid[3] = new pid_t(0.36f, 0.0006f, 0.0001f, 1.0f, 20.0f, 20, 10, 4);

    // Tracks
    for (auto &p : _track_speed_pid)
        p = new pid_t(0.02f, 0.0001f, 0.000001f, 0.5f, 10.0f, 30, 10, 4);

    // Legs
    for (auto &p : _leg_position_pid)
        p = new pid_t(10.0f, 0.005f, 0.008f, 0.5f, 10.0f, 20, 10, 4);
    for (auto &p : _leg_rotate_pid)
        p = new pid_t(10.0f, 0.005f, 0.008f, 0.5f, 10.0f, 20, 10, 4);

    _balance_pid = new pid_t(3.14f, 0.5f, 0.1f, 0.0f, 0.05f);

    // 6. 功率计
    _power_meter = new powermeter_drv_t(0x212, can_hub_t::can2);
    _power_data = new powermeter_data();
    if (_power_meter->init() == PYRO_OK) _power_flag = true;
}

// =========================================================
// 核心调度 (FSM Execute)
// =========================================================
void hybrid_chassis_t::_fsm_execute()
{
    // 直接访问基类双缓冲数组 _cmd 的当前读索引
    // 注意：模板基类使得 _cmd 类型已经是 hybrid_cmd_t，无需转换
    const auto &cmd = _cmd[_read_index];

    if (cmd.mode == hybrid_cmd_t::mode_t::ZERO_FORCE)
    {
        _reset_pids();
        std::memset(_mecanum_output_torque, 0, sizeof(_mecanum_output_torque));
        std::memset(_track_output_torque, 0, sizeof(_track_output_torque));
        std::memset(_leg_output_torque, 0, sizeof(_leg_output_torque));
        _send_motor_command(); // 发送 0 扭矩
    }
    else
    {
        _kinematics_solve();
        _chassis_control();
        _power_control();
        _send_motor_command();
    }
}

// =========================================================
// 反馈更新
// =========================================================
void hybrid_chassis_t::_update_feedback()
{
    for (auto *m : _mecanum_motor) m->update_feedback();
    for (auto *m : _track_motor) m->update_feedback();
    for (auto *m : _leg_motor) m->update_feedback();

    // 更新 RPM (注意 DJI 减速比)
    for (int i = 0; i < 4; ++i) {
        _current_wheel_rpm[i] = _radps_to_rpm(
            _mecanum_motor[i]->get_current_rotate() * dji_m3508_motor_drv_t::reciprocal_reduction_ratio
        );
    }
    for (int i = 0; i < 2; ++i) {
        _current_track_rpm[i] = _radps_to_rpm(_track_motor[i]->get_current_rotate());
        _current_leg_rad[i] = _leg_motor[i]->get_current_position();
        _current_leg_radps[i] = _leg_motor[i]->get_current_rotate();
    }

    if (!_power_flag) {
        if (_power_meter->init() == PYRO_OK) _power_flag = true;
    } else {
        _power_meter->get_data(*_power_data);
        _power = _power_data->power;
    }
}

// =========================================================
// 运动学解算
// =========================================================
void hybrid_chassis_t::_kinematics_solve()
{
    const auto &cmd = _cmd[_read_index];

    // 1. 底盘平面速度
    _solved_speeds_mps = _kinematics->solve(cmd.vx, cmd.vy, cmd.wz, cmd.drive_mode);

    // 转换为 RPM
    _target_wheel_rpm[0] =  _mps_to_rpm(_solved_speeds_mps.mec_fl, MEC_WHEEL_RADIUS_M);
    _target_wheel_rpm[1] = -_mps_to_rpm(_solved_speeds_mps.mec_fr, MEC_WHEEL_RADIUS_M);
    _target_wheel_rpm[2] =  _mps_to_rpm(_solved_speeds_mps.mec_bl, MEC_WHEEL_RADIUS_M);
    _target_wheel_rpm[3] = -_mps_to_rpm(_solved_speeds_mps.mec_br, MEC_WHEEL_RADIUS_M);

    _target_track_rpm[0] =  0.75f * _mps_to_rpm(_solved_speeds_mps.track_l, TRACK_WHEEL_RADIUS_M);
    _target_track_rpm[1] = -0.75f * _mps_to_rpm(_solved_speeds_mps.track_r, TRACK_WHEEL_RADIUS_M);

    // 2. 腿部位置控制 (旧逻辑移植：使用 wy 控制高度)
    if (cmd.leg_contract_mode == 0)
    {
        _target_leg_rad[0] += cmd.wy;
        _target_leg_rad[1] += -cmd.wy;

        // 限位
        auto clamp = [](float &v, float min, float max) {
            if(v > max) v = max; if(v < min) v = min;
        };
        clamp(_target_leg_rad[0], -0.4f, LEG_EXTEND_POS);
        clamp(_target_leg_rad[1], -LEG_EXTEND_POS, 0.4f);
    }
}

// =========================================================
// 控制计算 (PID)
// =========================================================
void hybrid_chassis_t::_chassis_control()
{
    const auto &cmd = _cmd[_read_index];

    // 1. 麦轮 PID
    for (int i = 0; i < 4; ++i) {
        _mecanum_output_torque[i] = _mecanum_speed_pid[i]->calculate(
            _target_wheel_rpm[i], _current_wheel_rpm[i]
        );
    }

    // 2. 腿部串级 PID
    for (int i = 0; i < 2; ++i) {
        _target_leg_radps[i] = _leg_position_pid[i]->calculate(
            _target_leg_rad[i], _current_leg_rad[i]
        );
        _leg_output_torque[i] = _leg_rotate_pid[i]->calculate(
            _target_leg_radps[i], _current_leg_radps[i]
        );
    }

    // 3. 履带 PID (仅在爬坡模式启用)
    if (cmd.drive_mode == hybrid_kin_t::drive_mode_t::CLIMBING) {
        for (int i = 0; i < 2; ++i) {
            _track_output_torque[i] = _track_speed_pid[i]->calculate(
                _target_track_rpm[i], _current_track_rpm[i]
            );
        }
    } else {
        _track_output_torque[0] = 0.0f;
        _track_output_torque[1] = 0.0f;
    }
}

// =========================================================
// 功率控制 & 发送
// =========================================================
void hybrid_chassis_t::_power_control()
{
    // TODO: 实现功率限制算法
}

void hybrid_chassis_t::_send_motor_command()
{
    // 简单的 Enable 检查
    auto check_enable = [](motor_base_t* m) { if(!m->is_enable()) m->enable(); };
    for (auto* m : _mecanum_motor) check_enable(m);
    for (auto* m : _track_motor) check_enable(m);
    for (auto* m : _leg_motor) check_enable(m);

    for (int i=0; i<4; ++i)
        _mecanum_motor[i]->send_torque(_mecanum_output_torque[i]);

    for (int i=0; i<2; ++i) {
        _track_motor[i]->send_torque(_track_output_torque[i]);
        _leg_motor[i]->send_torque(_leg_output_torque[i]);
    }
}

// =========================================================
// 辅助函数
// =========================================================
void hybrid_chassis_t::_reset_pids()
{
    for(auto* p : _mecanum_speed_pid) p->clear();
    for(auto* p : _track_speed_pid) p->clear();
    for(auto* p : _leg_position_pid) p->clear();
    for(auto* p : _leg_rotate_pid) p->clear();
    if(_balance_pid) _balance_pid->clear();
}

float hybrid_chassis_t::_mps_to_rpm(float mps, float radius)
{
    return (std::abs(radius) < 1e-6f) ? 0.0f : (mps / radius) * 9.5492966f;
}

float hybrid_chassis_t::_radps_to_rpm(float radps)
{
    return radps * 9.5492966f;
}

} // namespace pyro