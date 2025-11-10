#ifndef __PYRO_ROLL_DRV_H__
#define __PYRO_ROLL_DRV_H__

#include "pyro_dm_motor_drv.h"
#include "pyro_pid_ctrl.h"

namespace pyro
{
class roll_drv_t
{
public:
    roll_drv_t(motor_base_t *motor_base,
              const pid_ctrl_t &speed_pid,
              const pid_ctrl_t &position_pid
            );
    ~roll_drv_t()
    {
    }

    void set_radian(float target_radian);
    void set_dt(float dt);
    float get_current_radian();
    float get_current_rotate();
    void zero_force();
    void update_feedback();
    void set_torque(float torque);
    pid_ctrl_t speed_pid;
    pid_ctrl_t position_pid;


private:
    motor_base_t *_motor_base;
    float _current_radian;
    float _target_radian;
    float _current_rotate;
    float _dt = 0.001f;
    
};







} // namespace pyro


#endif
