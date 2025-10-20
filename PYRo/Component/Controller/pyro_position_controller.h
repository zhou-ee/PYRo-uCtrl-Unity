#ifndef __POSITION_CONTROLLER_H__
#define __POSITION_CONTROLLER_H__

#include "pyro_closed_controller.h"
#include "pyro_pid_ctrl.h"

namespace pyro
{

class position_controller_t : public closed_controller_t
{
    public:
        position_controller_t(motor_base_t* motor, pid_ctrl_t* pos_pid, pid_ctrl_t* rot_pid);
        float set_target(float target) override;
        virtual void update() override;
        void control(float dt) override;
        void set_upper_limit(float upper_limit);
        void set_lower_limit(float lower_limit);
    protected:
        pid_ctrl_t* _pos_pid;
        pid_ctrl_t* _rot_pid;

        float _target_pos;
        float _target_rot;

        float _feedback_pos;
        float _feedback_rot;

        float _control_value;

        float _upper_limit;
        float _lower_limit;
};

};

#endif