#ifndef __PYRO_WHEEL_DRV_H__
#define __PYRO_WHEEL_DRV_H__

#include "pyro_dji_motor_drv.h"
#include "pyro_pid_ctrl.h"
#include "pyro_dr16_rc_drv.h"

namespace pyro
{
class pyro_wheel_drv_t
{
  public:
    pyro_wheel_drv_t(motor_base_t *motor_base,    rc_drv_t *rc_drv,
                     const pid_ctrl_t &speed_pid, float radius);
    ~pyro_wheel_drv_t()
    {
    }

    void set_gear_ratio(float gear_ratio);
    void set_speed(float target_speed);
    void get_mode(rc_drv_t *rc_drv);
    float get_target_speed();
    float get_current_speed();
    motor_base_t *motor_base;

  private:
    float _rpm_to_mps(float rpm);
    pid_ctrl_t _speed_pid;
    float _radius;
    float _target_speed;
    float _current_speed;
    float _gear_ratio;

    
};
};

#endif

