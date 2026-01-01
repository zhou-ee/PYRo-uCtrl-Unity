#include "pyro_chassis_base.h"
#include "pyro_hybrid_chassis.h"
#include "pyro_mutex.h"
#include "pyro_rc_hub.h"

pyro::hybrid_chassis_t *hybrid_chassis_ptr = nullptr;
pyro::hybrid_cmd_t *hybrid_cmd_ptr         = nullptr;

extern "C"
{
    void chassis_rc2cmd(void const *rc_ctrl)
    {
        pyro::scoped_mutex_t lock(hybrid_chassis_ptr->get_mutex());
        auto *p_ctrl =
            static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(rc_ctrl);

        // 1. 检查右侧开关状态，如果是 DOWN，则进入 ZERO_FORCE 模式
        if (pyro::dr16_drv_t::sw_state_t::SW_MID != p_ctrl->rc.s_r.state)
        {
            hybrid_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ZERO_FORCE;
            hybrid_cmd_ptr->vx   = 0;
            hybrid_cmd_ptr->vy   = 0;
            hybrid_cmd_ptr->wz   = 0;
            hybrid_cmd_ptr->wy   = 0;
            return;
        }
        hybrid_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
        // 3. 映射摇杆数据
        // 注意：根据旧代码，vx = -Right_Y, vy = Right_X, wz = -Left_X, wy = Left_Y * 0.002
        // 新接口中，通道名称为 ch_rx, ch_ry, ch_lx, ch_ly
        hybrid_cmd_ptr->vx = -p_ctrl->rc.ch_ry;
        hybrid_cmd_ptr->vy = p_ctrl->rc.ch_rx;
        hybrid_cmd_ptr->wz = -p_ctrl->rc.ch_lx;
        hybrid_cmd_ptr->wy = p_ctrl->rc.ch_ly * 0.002f;

        // 4. 检查左侧开关状态，决定 drive_mode (CRUISING / CLIMBING)
        if (pyro::dr16_drv_t::sw_state_t::SW_UP == p_ctrl->rc.s_l.state)
        {
            hybrid_cmd_ptr->jump_mode = 1;
        }
        else if (pyro::dr16_drv_t::sw_state_t::SW_MID == p_ctrl->rc.s_l.state)
        {
            hybrid_cmd_ptr->drive_mode =
                pyro::hybrid_kin_t::drive_mode_t::CLIMBING;
        }
        else
        {
            hybrid_cmd_ptr->drive_mode =
                pyro::hybrid_kin_t::drive_mode_t::CRUISING;
        }


    }

    void start_app_thread(void *argument)
    {
        hybrid_chassis_ptr->start();
        while (true)
        {
            chassis_rc2cmd(pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)->read());
            vTaskDelay(1);
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