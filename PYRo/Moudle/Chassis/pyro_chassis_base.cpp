#include "pyro_chassis_base.h"

#include <thread>

extern "C" void chassis_task(void *argument);

namespace pyro
{

chassis_base_t::chassis_base_t()
{
    xTaskCreate(chassis_task, "chassis_task", configMINIMAL_STACK_SIZE, this,
                tskIDLE_PRIORITY + 1, &_chassis_task_handle);
}
chassis_base_t::chassis_base_t(type_t type)
{
    xTaskCreate(chassis_task, "chassis_task", configMINIMAL_STACK_SIZE, this,
                tskIDLE_PRIORITY + 1, &_chassis_task_handle);
}

void chassis_base_t::thread()
{
    update_feedback();
    kinematics_solve();
    chassis_control();
    power_control();
    send_motor_command();
};

} // namespace pyro

extern "C" void chassis_task(void *argument)
{
    auto *chassis = static_cast<pyro::chassis_base_t *>(argument);
    if (chassis)
    {
        while (true)
        {
            chassis->thread();
        }
    }
}