#include "pyro_core_config.h"
#ifdef RC_DEMO_EN
#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_uart_drv.h"

#ifdef __cplusplus

extern "C"
{
    pyro::rc_drv_t *dr16_drv;
    void pyro_rc_demo(void *arg)
    {
        pyro::get_uart5().enable_rx_dma();
        dr16_drv = new pyro::dr16_drv_t(&pyro::get_uart5());
        dr16_drv->init();
        dr16_drv->enable();
        static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            dr16_drv->get_p_ctrl());
        static auto *p_last_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(
            dr16_drv->get_p_last_ctrl());
        vTaskDelete(nullptr);
    }
}
#endif
#endif
