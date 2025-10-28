#include "pyro_shoot_base.h"

namespace pyro
{
shoot_base_t::shoot_base_t(rc_drv_t *rc_drv)
{
    rc_drv->set_get_mode([this](rc_drv_t *rc_drv) -> void { get_mode(rc_drv); });
}

void shoot_base_t::get_mode(rc_drv_t *rc_drv)
{
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
        rc_drv->get_p_ctrl());
    static auto *p_last_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
        rc_drv->get_p_last_ctrl());
}


}