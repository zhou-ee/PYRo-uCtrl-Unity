#ifndef __PYRO_CHASSIS_DRV_H__
#define __PYRO_CHASSIS_DRV_H__

#include "pyro_steering_wheel_drv.h"
#include "pyro_power_control_drv.h"
#include "pyro_rc_hub.h"
#include <array>

#define POWER_CONTROL_USE 1
#define POWER_LIMIT 70

namespace pyro
{
class chassis_drv_t
{
  public:
    enum total_mode_t
    {
        ZERO_FORCE         = 0x00,
        RC_CONTROL         = 0x01,
        GYRO_DIAL          = 0x02
    };

    chassis_drv_t(steering_wheel_drv_t *drv1,
                  steering_wheel_drv_t *drv2,
                  steering_wheel_drv_t *drv3,
                  steering_wheel_drv_t *drv4,
                  pid_t* yaw_pid,
                  uint8_t powercontrol_num
                  );
    ~chassis_drv_t()
    {
    }
    
    void dr16_cmd(void const *rc_ctrl);
    void vt03_cmd(void const *rc_ctrl);
    void update_feedback();
    void zero_force();
    void chassis_control(float yaw_err);

  private:
    std::array<steering_wheel_drv_t*, 4> _steering_wheel_drv;
    pid_t *_yaw_pid;
    float _vx, _vy, _wz; // The direction of the front of the vehicle represents the vy direction
    uint8_t _s_right;
    uint8_t _powercontrol_num;
    std::vector<power_control_drv_t::wheel_data_t> _wheel_data;

    friend class vofa_drv_t;
};

}

#endif

