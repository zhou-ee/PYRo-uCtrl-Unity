#include "pyro_core_config.h"
#include "freertos.h"
#include "task.h"


extern "C" void pyro_rc_demo(void *arg);
extern "C" void start_demo_task(void const *argument)
{
#if DEMO_MODE

#if RC_DEMO_EN
    xTaskCreate(pyro_rc_demo, "pyro_rc_demo", 128, nullptr, configMAX_PRIORITIES - 2, nullptr);
#endif

#endif
    vTaskDelete(nullptr);
}
