#include "pyro_core_config.h"
#if RC_DEMO_EN
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_rc_hub.h"

#ifdef __cplusplus

extern "C"
{
    void dr16_cmd(void const *rc_ctrl)
    {
        // Example
        static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const*>(rc_ctrl);
    }
    void vt03_cmd(void const *rc_ctrl)
    {
        // Example
        static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t const*>(rc_ctrl);
    }
    pyro::rc_drv_t *dr16_drv;
    pyro::rc_drv_t *vt03_drv;
    void pyro_rc_demo(void *arg)
    {
        // Initialize the DR16 RC driver
        // init_thread already did this part
        // User can customize the initialization as needed
        dr16_drv = pyro::rc_hub_t::get_instance(
            pyro::rc_hub_t::DR16);
        dr16_drv->init();
        dr16_drv->enable();

        vt03_drv = pyro::rc_hub_t::get_instance(
            pyro::rc_hub_t::VT03);
        vt03_drv->init();
        vt03_drv->enable();

        // if dr16_cmd is a member function, use a lambda to bind 'this'
        dr16_drv->config_rc_cmd(dr16_cmd);
        vt03_drv->config_rc_cmd(vt03_cmd);

        vTaskDelete(nullptr);
    }
}
#endif
#endif
