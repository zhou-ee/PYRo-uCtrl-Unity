#ifndef __PYRO_PITCH_DRV_H__
#define __PYRO_PITCH_DRV_H__

#include "pyro_motor_base.h"
#include "pyro_algo_pid.h"

namespace pyro
{
class pitch_drv_t
{
public:
    pitch_drv_t(motor_base_t *motor_base,
              pid_t rotate_pid,
              pid_t position_pid);
    ~pitch_drv_t()
    {
    }

    void set_offset_radian(float offset_radian);
    void update_target_radian(float diff_radian);
    void zero_force();
    void update_feedback(float real_angle, float real_speed);
    void calculate_torque();
    void output_torque() const;
    motor_base_t *motor_base;

    pid_t _pitch_rotate_pid;
    pid_t _pitch_position_pid;
    float _offset_radian{};
    float _target_radian{};
    float _current_radian{};
    float _target_rotate{};
    float _current_rotate{};
    float _output_torque{};
};


}

#endif