#ifndef __PYRO_CHASSIS_BASE_H__
#define __PYRO_CHASSIS_BASE_H__

namespace pyro
{
class chassis_base_drv_t
{
    public:
    virtual void control(float dt) = 0;
    virtual void set_target(float vx, float vy, float wz) = 0;
    virtual void update() = 0;
    virtual void zero_force() = 0;
};
};

#endif