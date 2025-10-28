#ifndef __PYRO_TRIGGER_DRV_H__
#define __PYRO_TRIGGER_DRV_H__ 

#include "pyro_dji_motor_drv.h"
#include "pyro_pid_ctrl.h"
#include "pyro_vofa.h"

namespace pyro
{ 
class trigger_drv_t 
{ 
public:
    trigger_drv_t(motor_base_t *motor_base,
                  const pid_ctrl_t &_rotate_pid, 
                  const pid_ctrl_t &_position_pid,
                  float step_radian
                );
    ~trigger_drv_t()
    {
    }
    void set_dt(float dt);
    void set_gear_ratio(float gear_ratio);
    void set_rotate(float target_rotate);
    void set_radian(float target_radian);
    float get_rotate();
    float get_radian();
    void zero_force();
    void update_feedback();
    motor_base_t *motor_base;

private:
    enum trigger_mode_t
    {
        ROTATE      = 0x00,
        POSITION    = 0x01
    };
    pid_ctrl_t _rotate_pid;
    pid_ctrl_t _position_pid;
    mode_t _mode;
    float _dt = 0.001f;
    float _step_radian;
    float _target_rotate;
    float _current_rotate;
    float _target_radian;
    float _current_radian;
    float _gear_ratio = 1;

friend class vofa_drv_t;
};

}
#endif
