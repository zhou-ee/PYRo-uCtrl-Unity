#ifndef __PYRO_CHASSIS_DRV_H__
#define __PYRO_CHASSIS_DRV_H__

#include "pyro_steering_wheel_drv.h"
#include "pyro_rc_hub.h"

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

    chassis_drv_t(steering_wheel_drv_t *steering_wheel_drv_1,
                  steering_wheel_drv_t *steering_wheel_drv_2,
                  steering_wheel_drv_t *steering_wheel_drv_3,
                  steering_wheel_drv_t *steering_wheel_drv_4,
                  wheel_drv_t *wheel_drv_1,
                  wheel_drv_t *wheel_drv_2,
                  wheel_drv_t *wheel_drv_3,
                  wheel_drv_t *wheel_drv_4,
                  pid_ctrl_t* yaw_pid
                  );
    ~chassis_drv_t()
    {
    }
    
    void dr16_cmd(void const *rc_ctrl);
    void vt03_cmd(void const *rc_ctrl);
    void update_feedback();
    void zero_force();
    void chassis_control(float yaw_err);

    steering_wheel_drv_t *_steering_wheel_drv_1;
    steering_wheel_drv_t *_steering_wheel_drv_2;
    steering_wheel_drv_t *_steering_wheel_drv_3;
    steering_wheel_drv_t *_steering_wheel_drv_4;

    wheel_drv_t *_wheel_drv_1;
    wheel_drv_t *_wheel_drv_2;
    wheel_drv_t *_wheel_drv_3;
    wheel_drv_t *_wheel_drv_4;

    pid_ctrl_t *_yaw_pid;



  private:
  float _vx, _vy, _wz; // The direction of the front of the vehicle represents the vy direction
  uint8_t _s_right;

};

}

#endif

