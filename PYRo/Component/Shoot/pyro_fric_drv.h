#ifndef __PYRO_FRIC_H__
#define __PYRO_FRIC_H__

#include "pyro_dji_motor_drv.h"
#include "pyro_pid_ctrl.h"

namespace pyro
{
class fric_drv_t
{
public:
    fric_drv_t(motor_base_t *motor_base,
               const pid_ctrl_t &speed_pid, float radius);
    ~fric_drv_t()
    {
    }

    void set_speed(float target_speed);
    void zero_force();
    void fric_control();
    void update_feedback();
    motor_base_t *motor_base;

private:
    pid_ctrl_t _speed_pid;
    float _radius;
    float _target_speed;
    float _current_speed;
};


}
#endif

