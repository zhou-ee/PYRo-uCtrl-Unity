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
    void step_forward();
    float get_rotate();
    float get_radian();
    float get_target_radian();
    float get_step_radian();
    void zero_force();
    void update_feedback();
    void control();
    motor_base_t *motor_base;

private:
    enum trigger_mode_t
    {
        ROTATE      = 0x00,
        POSITION    = 0x01
    };

    void _motor_to_trigger_radian();
    // float _update_trigger_radian();

    pid_ctrl_t _rotate_pid;
    pid_ctrl_t _position_pid;
    trigger_mode_t _mode{};
    float _dt = 0.001f;
    float _step_radian{};
    float _target_trigger_rotate{};
    float _current_trigger_rotate{};
    // float _target_trigger_radian{};
    // float _current_trigger_radian{};
    float _current_trigger_total_radian;
    float _target_trigger_total_radian;
    float _current_motor_radian{};
    float _last_motor_radian{};
    // float _last_motor_radian;
    // uint8_t _motor_total_count = 0;
    float _gear_ratio = 1;
    bool _is_first_update = true;

    float _test_rotate_cmd{};
    float _test_torque_cmd{};

friend class vofa_drv_t;
};

}
#endif
