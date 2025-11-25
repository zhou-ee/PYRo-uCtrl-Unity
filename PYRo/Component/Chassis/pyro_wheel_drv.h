#ifndef __PYRO_WHEEL_DRV_H__
#define __PYRO_WHEEL_DRV_H__

#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"

namespace pyro
{
class wheel_drv_t
{
  public:
    wheel_drv_t(motor_base_t *motor_base,
                     const pid_t &speed_pid, float radius);
    ~wheel_drv_t()
    {
    }

    void set_gear_ratio(float gear_ratio);
    void set_speed(float target_speed);
    void zero_force();
    float get_target_speed();
    float get_current_speed();
    float get_current_motor_rotate();
    void update_feedback();
    void control();

    float torque_cmd;
    float rotate;
    float last_torque;

    motor_base_t *motor_base;

  private:
    pid_t _speed_pid;
    float _radius;
    float _target_speed;
    float _current_speed;
    float _gear_ratio;


    friend class vofa_drv_t;
};
};

#endif

