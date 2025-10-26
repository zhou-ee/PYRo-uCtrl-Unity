#ifndef __PYRO_TRIGGER_DRV_H__
#define __PYRO_TRIGGER_DRV_H__ 

#include "pyro_dji_motor_drv.h"
#include "pyro_pid_ctrl.h"

namespace pyro
{ 
class trigger_drv_t 
{ 
public:
    trigger_drv_t(motor_base_t *motor_base,
                  const pid_ctrl_t &_rotate_pid, 
                  const pid_ctrl_t &position_pid);
    ~trigger_drv_t()
    {
    }

    void set_gear_ratio(float gear_ratio);
    void step_fire();
    void continue_fire();
    void zero_force();
    void set_rotate(float target_speed);
    void update_feedback();
    motor_base_t *motor_base;

private:
    pid_ctrl_t _rotate_pid;
    pid_ctrl_t _position_pid;
    float _target_rotate;
    float _current_rotate;
    float _gear_ratio;
    uint8_t _block_count1 = 0;
    uint8_t _block_count2 = 0;
};

}
#endif
