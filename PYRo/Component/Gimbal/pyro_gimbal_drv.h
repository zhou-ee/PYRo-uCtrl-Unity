#ifndef __PYRO_GIMBAL_DRV_H__
#define __PYRO_GIMBAL_DRV_H__

#include "pyro_pitch_drv.h"
#include "pyro_roll_drv.h"
#include "pyro_yaw_drv.h"
#include "pyro_rc_hub.h"
#include "IMU_Base.h"

namespace pyro
{
class gimbal_drv_t
{
public:
    enum total_mode_t
    {
        ZERO_FORCE         = 0x00,
        RC_CONTROL         = 0x01,
        AUTO_AIM_CONTROL   = 0x02
    };

    gimbal_drv_t(pitch_drv_t *pitch_drv, 
                 roll_drv_t *roll_drv, 
                 yaw_drv_t *yaw_drv,
                 IMU_obj *imu);
    ~gimbal_drv_t()
    {
    }

    void set_dt(float dt);
    void update_feedback();
    void zero_force();
    void dr16_cmd(void const *rc_ctrl);
    void vt03_cmd();
    void set_control();
    void control();

private:
    pitch_drv_t *_pitch_drv{};
    roll_drv_t *_roll_drv{};
    yaw_drv_t *_yaw_drv{};
    IMU_obj *_imu{};
    total_mode_t _total_mode;
    float _dt = 0.001f;
    bool _is_init = true;

    float _imu_yaw_offset_radian{};
    float _target_yaw_imu_radian{};
    float _current_yaw_imu_radian{};

    float _target_pitch_imu_radian{};
    float _current_pitch_imu_radian{};

    float _current_roll_imu_radian{};
    
    float _yaw_speed_cmd{};
    float _yaw_cmd_torque{};
    float _pitch_cmd_torque{};
    float _roll_cmd_torque{};
};

}

#endif
