#ifndef __PYRO_HYBRID_CHASSIS_H__
#define __PYRO_HYBRID_CHASSIS_H__

// 引入新的模板基类
#include "pyro_chassis_base.h"

// 引入原有的算法和硬件驱动
#include "pyro_algo_pid.h"
#include "pyro_kin_hybrid.h"
#include "pyro_motor_base.h"
#include "pyro_powermeter.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"

namespace pyro
{

// =========================================================
// 1. 定义混合底盘专用命令 (继承自 cmd_base_t)
// =========================================================
struct hybrid_cmd_t : cmd_base_t
{
    // vx, vy, wz, timestamp 已由基类提供

    // 业务模式
    enum class mode_t : uint8_t {
        ZERO_FORCE, // 无力/掉电
        ACTIVE      // 正常工作
    } mode;

    hybrid_kin_t::drive_mode_t drive_mode; // 巡航 or 爬坡
    uint8_t leg_contract_mode;             // 腿部动作模式
    float wy;

    hybrid_cmd_t()
        : cmd_base_t(), mode(mode_t::ZERO_FORCE),
          drive_mode(hybrid_kin_t::drive_mode_t::CRUISING),
          leg_contract_mode(0), wy(0)
    {
    }
};

// =========================================================
// 2. 混合底盘类定义 (CRTP + Singleton)
// =========================================================
class hybrid_chassis_t final : public chassis_base_t<hybrid_chassis_t, hybrid_cmd_t>
{
    // 必须声明基类为友元，否则基类无法访问私有构造函数来创建单例
    friend class chassis_base_t<hybrid_chassis_t, hybrid_cmd_t>;

  public:
    // 删除拷贝/移动 (基类已处理，这里再次强调或省略均可)
    hybrid_chassis_t(const hybrid_chassis_t &) = delete;
    hybrid_chassis_t &operator=(const hybrid_chassis_t &) = delete;

  protected:
    // -----------------------------------------------------
    // 构造函数私有化 (Singleton)
    // -----------------------------------------------------
    hybrid_chassis_t();
    ~hybrid_chassis_t() override;

    // -----------------------------------------------------
    // 基类虚函数实现
    // -----------------------------------------------------
    void _init() override;
    void _fsm_execute() override;
    void _update_feedback() override;
    void _kinematics_solve() override;
    void _chassis_control() override;
    void _power_control() override;
    void _send_motor_command() override;

  private:
    // -----------------------------------------------------
    // 私有成员变量与辅助函数
    // -----------------------------------------------------
    void _update_imu_data(float pitch_rad, float roll_rad, float yaw_rad);
    void _reset_pids();
    static float _mps_to_rpm(float mps, float radius);
    static float _radps_to_rpm(float radps);

    // --- 算法对象 ---
    hybrid_kin_t *_kinematics{nullptr};

    // --- 传感器 ---
    powermeter_drv_t *_power_meter{nullptr};
    powermeter_data *_power_data{nullptr};
    float _power{0.0f};
    bool _power_flag{false};

    // --- 状态反馈 ---
    float _current_pitch_rad = 0.0f;
    float _current_roll_rad  = 0.0f;
    float _current_yaw_rad   = 0.0f;

    // --- 目标与实测值 ---
    hybrid_kin_t::hybrid_speeds_t _solved_speeds_mps{};
    float _target_wheel_rpm[4]{};
    float _target_track_rpm[2]{};
    float _target_leg_rad[2]{};
    float _target_leg_radps[2]{};

    float _current_wheel_rpm[4]{};
    float _current_track_rpm[2]{};
    float _current_leg_rad[2]{};
    float _current_leg_radps[2]{};

    // --- 输出 ---
    float _mecanum_output_torque[4]{};
    float _track_output_torque[2]{};
    float _leg_output_torque[2]{};

    // --- 硬件句柄 ---
    motor_base_t *_mecanum_motor[4]{nullptr};
    motor_base_t *_track_motor[2]{nullptr};
    motor_base_t *_leg_motor[2]{nullptr};

    // --- PID ---
    pid_t *_mecanum_speed_pid[4]{nullptr};
    pid_t *_track_speed_pid[2]{nullptr};
    pid_t *_balance_pid{nullptr};
    pid_t *_leg_position_pid[2]{nullptr};
    pid_t *_leg_rotate_pid[2]{nullptr};

    // --- 常量配置 ---
    static constexpr float MEC_WHEEL_RADIUS_M   = 0.076f;
    static constexpr float TRACK_WHEEL_RADIUS_M = 0.035f;
    static constexpr float LEG_EXTEND_POS       = 1.60f;
};

} // namespace pyro

#endif // __PYRO_HYBRID_CHASSIS_H__