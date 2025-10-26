#ifndef __PYRO_SHOOT_DRV_H__
#define __PYRO_SHOOT_DRV_H__ 

#include "pyro_trigger_drv.h"
#include "pyro_fric_drv.h"
#include "pyro_dr16_rc_drv.h"

namespace pyro
{ 
class shoot_drv_t
{
public:
    shoot_drv_t(fric_drv_t *fric_drv_1, 
                 fric_drv_t *fric_drv_2, 
                 trigger_drv_t *trigger_drv,
                 rc_drv_t *rc_drv
                );
    ~shoot_drv_t()
    {
    }
    
    void get_mode(rc_drv_t *rc_drv);
    void update_feedback();
    void zero_force();
    void shoot_control();

    fric_drv_t *fric_drv_1;
    fric_drv_t *fric_drv_2;
    trigger_drv_t *trigger_drv;

private:
    uint8_t _s_right, _s_left, _s_left_last;

};

}

#endif

