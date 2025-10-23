#ifndef __PYRO_WHEEL_DRV_H__
#define __PYRO_WHEEL_DRV_H__ 

#include "pyro_velocity_controller.h"

namespace pyro
{
class wheel_drv_t : public velocity_controller_t
{
    public:
        wheel_drv_t(motor_base_t* motor, pid_ctrl_t* spd_pid,float radius): velocity_controller_t(motor,spd_pid), _radius(radius){};
        void update() override;
        float _radius;
};
};


#endif