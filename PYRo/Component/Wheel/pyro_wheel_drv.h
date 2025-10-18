#ifndef __PYRO_WHEEL_DRV_H__
#define __PYRO_WHEEL_DRV_H__

#include "pyro_dji_motor_drv.h"
#include "pyro_pid_ctrl.h"
#include "pyro_dr16_rc_drv.h"

namespace pyro
{
class pyro_wheel_drv_t : public dji_m3508_motor_drv_t
{
  public:
    pyro_wheel_drv_t(pyro::dji_motor_tx_frame_t::register_id_t id,
                     can_hub_t::which_can which,
                     float radius,
                     const pid_ctrl_t &speed_pid,
                     rc_drv_t *rc_drv);
    ~pyro_wheel_drv_t()
    {
    }

    void set_rotate(float target_rotate);
    void get_mode(rc_drv_t *rc_drv);
    float get_target_rotate();

  private:
    float _radius;
    pid_ctrl_t _speed_pid;
    float _target_rotate;
    
};
};

#endif

