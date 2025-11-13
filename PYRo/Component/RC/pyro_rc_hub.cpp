#include "pyro_rc_hub.h"

namespace pyro
{
rc_drv_t *rc_hub_t::get_instance(which_rc_t which_rc)
{
    switch (which_rc)
    {
        case DR16:
        {
            static uart_drv_t *dr16_uart =
                uart_drv_t::get_instance(uart_drv_t::uart5);
            static dr16_drv_t dr16_rc_drv(dr16_uart);
            return &dr16_rc_drv;
        }
        case VT03:
        {
            static uart_drv_t *vt03_uart =
                uart_drv_t::get_instance(uart_drv_t::uart1);
            static vt03_drv_t vt03_rc_drv(vt03_uart);
            return &vt03_rc_drv;
        }
        default:;
    }
    return nullptr;
}

} // namespace pyro