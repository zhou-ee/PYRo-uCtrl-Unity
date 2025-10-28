#ifndef __PYRO_SHOOT_BASE_H__
#define __PYRO_SHOOT_BASE_H__ 

#include "pyro_trigger_drv.h"
#include "pyro_fric_drv.h"
#include "pyro_dr16_rc_drv.h"

namespace pyro
{ 
class shoot_base_t
{
public:
    enum total_mode_t
    {
        ZERO_FORCE         = 0x00,
        RC_CONTROL         = 0x01,
        AUTO_AIM_CONTROL   = 0x02
    };
    enum local_mode_t
    {
        SHOOT_STOP         = 0x00,
        SHOOT_SETUP        = 0x01,
        SHOOT_READY        = 0x02,
        SHOOT_START        = 0x03,
        SHOOT_WAIT         = 0x04,
        SHOOT_CONTINUOUS   = 0x05
    };
    shoot_base_t(rc_drv_t *rc_drv);
    ~shoot_base_t()
    {
    }
    
    void get_mode(rc_drv_t *rc_drv);
    virtual void update_feedback();
    virtual void zero_force();
    virtual void shoot_control();

private:
    total_mode_t _total_mode;
    local_mode_t _local_mode;
};

}

#endif

