/*
 * @Author: lucky zhou_20006ee@163.com
 * @Date: 2025-10-15 14:38:32
 * @LastEditors: lucky zhou_20006ee@163.com
 * @LastEditTime: 2025-10-15 15:03:04
 * @FilePath: \PYRo-uCtrl-Unity\PYRo\Application\Demo\demo_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "cmsis_os.h"
#include "pyro_core_config.h"
#include "task.h"


extern "C"
{
extern void pyro_rc_demo(void *arg);
extern void pyro_motor_demo(void *arg);
extern void pyro_wheel_demo(void *arg);
extern void pyro_controller_demo(void *arg);
extern void pyro_vofa_demo(void *arg);
extern void pyro_engineer_chassis_demo(void *arg);
extern void pyro_engineer_arm_demo(void *arg);
extern void init_task(void *arg);
extern void IMU_task(void * argument);
void start_demo_task(void const *argument)
{

     xTaskCreate(init_task, "init_task", 256, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
 //变成IMU_TASK 之后自己写一个demo
#if DEMO_MODE

#if RC_DEMO_EN
     xTaskCreate(pyro_rc_demo, "pyro_rc_demo", 128, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif
#if MOTOR_DEMO_EN
     xTaskCreate(pyro_motor_demo, "pyro_motor_demo", 512, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif
#if WHEEL_DEMO_EN
     xTaskCreate(pyro_wheel_demo, "pyro_wheel_demo", 512, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif
#if CONTROL_DEMO_EN
     xTaskCreate(pyro_control_demo, "pyro_control_demo", 512, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif


#if CONTROLLER_DEMO_EN
     xTaskCreate(pyro_controller_demo, "pyro_controller_demo", 512, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif

#if SHOOT_DEMO_EN
     xTaskCreate(pyro_shoot_demo, "pyro_shoot_demo", 512, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif
#if IMU_DEMO_EN
     xTaskCreate(IMU_task, "IMU_task", 512, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif


#if ENGINEER_CHASSIS_DEMO_EN
     xTaskCreate(pyro_engineer_chassis_demo, "pyro_engineer_chassis_demo", 512, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif

#if ENGINEER_ARM_DEMO_EN
     xTaskCreate(pyro_engineer_arm_demo, "pyro_engineer_arm_demo", 512, nullptr,
                 configMAX_PRIORITIES - 2, nullptr);
#endif

#endif
    vTaskDelete(nullptr);
}

}

#include "pyro_can_drv.h"
#include "pyro_uart_drv.h"

pyro::can_drv_t *can1_drv;
pyro::can_drv_t *can2_drv;
pyro::can_drv_t *can3_drv;
void init_task(void *arg)
{
     TaskHandle_t chassis_handle,arm_handle;

     vTaskDelay(500);
     while(xTaskGetHandle("pyro_engineer_chassis_demo") == NULL);
     while(xTaskGetHandle("pyro_engineer_arm_demo") == NULL);
     chassis_handle = xTaskGetHandle("pyro_engineer_chassis_demo");
     arm_handle = xTaskGetHandle("pyro_engineer_arm_demo");

     pyro::uart_drv_t::get_instance(pyro::uart5)->enable_rx_dma();

     pyro::can_hub_t::get_instance();
     can1_drv = new pyro::can_drv_t(&hfdcan1);
     can2_drv = new pyro::can_drv_t(&hfdcan2);
     can3_drv = new pyro::can_drv_t(&hfdcan3);

     can1_drv->init();
     can2_drv->init();
     can3_drv->init();
     can1_drv->start();
     can2_drv->start();  
     can3_drv->start();

     vTaskResume(chassis_handle);
     vTaskResume(arm_handle);
     vTaskDelete(nullptr);
}