#include "pyro_hybrid_chassis.h"

namespace pyro
{

void hybrid_chassis_t::fsm_active_t::state_cruising_t::on_enter(
    hybrid_chassis_t *owner)
{
    owner->_ctx.data.target_leg_rad[0] = 0.0f;
    owner->_ctx.data.target_leg_rad[1] = 0.0f;
}

void hybrid_chassis_t::fsm_active_t::state_cruising_t::on_execute(
    hybrid_chassis_t *owner)
{
    // 3. 运行闭环控制
    _chassis_control(&owner->_ctx);

    // 4. 输出到硬件
    _send_motor_command(&owner->_ctx);
}

void hybrid_chassis_t::fsm_active_t::state_cruising_t::exit(
    hybrid_chassis_t *owner)
{
    // 退出巡航模式
}

} // namespace pyro
