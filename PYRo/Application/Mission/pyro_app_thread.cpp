#include "pyro_chassis_base.h"
#include "pyro_hybrid_chassis.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"

pyro::hybrid_chassis_t *hybrid_chassis_ptr = nullptr;
pyro::hybrid_cmd_t *hybrid_cmd_ptr         = nullptr;

extern "C"
{
    // void chassis_rc2cmd(void const *rc_ctrl)
    // {
    //     pyro::scoped_mutex_t lock(hybrid_chassis_ptr->get_mutex());
    //     auto *p_ctrl =
    //         static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);
    //     if (pyro::dr16_drv_t::dr16_sw_state_t::DR16_SW_DOWN ==
    //         p_ctrl->rc.s[pyro::dr16_drv_t::DR16_SW_RIGHT].state)
    //     {
    //         hybrid_cmd_ptr->mode = pyro::chassis_base_t::mode_t::ZERO_FORCE;
    //         hybrid_cmd_ptr->vx   = 0;
    //         hybrid_cmd_ptr->vy   = 0;
    //         hybrid_cmd_ptr->wz   = 0;
    //         hybrid_cmd_ptr->wy   = 0;
    //         return;
    //     }
    //     hybrid_cmd_ptr->mode = pyro::chassis_base_t::mode_t::ACTIVE;
    //     hybrid_cmd_ptr->vx =
    //     -p_ctrl->rc.ch[pyro::dr16_drv_t::DR16_CH_RIGHT_Y]; hybrid_cmd_ptr->vy
    //     = p_ctrl->rc.ch[pyro::dr16_drv_t::DR16_CH_RIGHT_X];
    //     hybrid_cmd_ptr->wz   =
    //     - p_ctrl->rc.ch[pyro::dr16_drv_t::DR16_CH_LEFT_X];
    //     hybrid_cmd_ptr->wy =
    //         p_ctrl->rc.ch[pyro::dr16_drv_t::DR16_CH_LEFT_Y] * 0.002f;
    //     // hybrid_cmd_ptr->wz =
    //     p_ctrl->rc.ch[pyro::dr16_drv_t::DR16_CH_LEFT_X];
    //     // hybrid_cmd_ptr->wz = 0;
    //     if (pyro::dr16_drv_t::dr16_sw_state_t::DR16_SW_UP ==
    //         p_ctrl->rc.s[pyro::dr16_drv_t::DR16_SW_LEFT].state)
    //     {
    //         hybrid_cmd_ptr->drive_mode =
    //             pyro::hybrid_kin_t::drive_mode_t::CLIMBING;
    //     }
    //     else
    //     {
    //         hybrid_cmd_ptr->drive_mode =
    //             pyro::hybrid_kin_t::drive_mode_t::CRUISING;
    //     }
    //     if (pyro::dr16_drv_t::dr16_sw_state_t::DR16_SW_UP ==
    //         p_ctrl->rc.s[pyro::dr16_drv_t::DR16_SW_RIGHT].state)
    //     {
    //         hybrid_cmd_ptr->leg_contract_mode = 1;
    //     }
    //     else
    //     {
    //         hybrid_cmd_ptr->leg_contract_mode = 0;
    //     }
    // }
    void start_app_thread(void *argument)
    {
        hybrid_chassis_ptr->start();
        while (true)
        {
        }
    }

    void pyro_app_init_thread(void *argument)
    {
        // Initialize Hybrid Chassis
        hybrid_chassis_ptr = pyro::hybrid_chassis_t::instance();
        hybrid_cmd_ptr     = new pyro::hybrid_cmd_t();
        xTaskCreate(start_app_thread, "start_app_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
    }


}