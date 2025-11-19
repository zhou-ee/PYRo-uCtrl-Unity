#ifndef __PYRO_CHASSIS_BASE_H__
#define __PYRO_CHASSIS_BASE_H__


class chassis_base_t
{
  public:
    chassis_base_t()                = default;
    virtual void update_feedback()  = 0;
    virtual void kinematics_solve() = 0;
    virtual void chassis_control()  = 0;
    virtual void power_control()    = 0;
    virtual void send_command()     = 0;
    virtual ~chassis_base_t()       = default;
};


#endif
