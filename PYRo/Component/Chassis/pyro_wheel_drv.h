#ifndef __PYRO_WHEEL_DRV_H__
#define __PYRO_WHEEL_DRV_H__

#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"
#include "pyro_power_control_drv.h"

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
    float *get_p_target_speed();
    float get_current_speed();
    float *get_p_current_speed();
    float get_current_motor_rotate();
    void update_feedback();
    void control();

    float torque_cmd;
    float rotate;

    motor_base_t *motor_base;
    power_control_drv_t power_control_drv;


  private:
    pid_t _speed_pid;
    float _radius;
    float _target_speed;
    float _current_speed;
    float _gear_ratio;
};
};

#endif

