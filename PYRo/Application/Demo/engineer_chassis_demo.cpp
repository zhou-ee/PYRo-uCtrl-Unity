#include "pyro_core_config.h"


#if ENGINEER_CHASSIS_DEMO_EN

#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"

#include "pyro_velocity_controller.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_pid_ctrl.h"

#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_uart_drv.h"

#include "pyro_macunum_chassis_drv.h"

extern "C"
{

typedef enum 
{
    ZERO_FORCE,
    RC_CONTROL
}
mode_t;

mode_t mode = ZERO_FORCE;
float vx_rc = 0.0f;
float vy_rc = 0.0f;
float wz_rc = 0.0f;

pyro::rc_drv_t *dr16_drv;
void get_mode(pyro::rc_drv_t *rc_drv)
{
    static auto *p_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->get_p_ctrl());
    static auto *p_last_ctrl = static_cast<pyro::dr16_drv_t::dr16_ctrl_t *>(dr16_drv->get_p_last_ctrl());

    if(RC_SW_UP == p_ctrl->rc.s[0])
    {
        mode = ZERO_FORCE;
    }
    else if(RC_SW_UP != p_ctrl->rc.s[0])
    {
        mode = RC_CONTROL;
    }

    vx_rc = static_cast<float>(p_ctrl->rc.ch[0]) / 660.0f ;
    vy_rc = static_cast<float>(p_ctrl->rc.ch[1]) / 660.0f ;
    wz_rc = static_cast<float>(p_ctrl->rc.ch[2]) / 660.0f *5;
}



pyro::can_drv_t *can1_drv;
pyro::can_drv_t *can2_drv;
pyro::can_drv_t *can3_drv;

pyro::motor_base_t *wheel_motor1;
pyro::pid_ctrl_t *wheel_1_spd_pid;
pyro::motor_base_t *wheel_motor2;
pyro::pid_ctrl_t *wheel_2_spd_pid;
pyro::motor_base_t *wheel_motor3;
pyro::pid_ctrl_t *wheel_3_spd_pid;
pyro::motor_base_t *wheel_motor4;
pyro::pid_ctrl_t *wheel_4_spd_pid;

pyro::wheel_drv_t *wheel_drv1;
pyro::wheel_drv_t *wheel_drv2;
pyro::wheel_drv_t *wheel_drv3;
pyro::wheel_drv_t *wheel_drv4;

pyro::macunum_chassis_drv_t *chassis_drv;




void pyro_engineer_chassis_demo(void *arg)
{
        pyro::uart_drv_t::get_instance(pyro::uart5)->enable_rx_dma();
        //pyro::get_uart5().enable_rx_dma();
        dr16_drv = new pyro::dr16_drv_t(pyro::uart_drv_t::get_instance(pyro::uart5));
        dr16_drv->init();
        dr16_drv->enable();

        dr16_drv->set_get_mode(get_mode);


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

        wheel_motor1 = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can1);
        wheel_motor2 = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_2, pyro::can_hub_t::can1);
        wheel_motor3 = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can1);
        wheel_motor4 = new pyro::dji_m3508_motor_drv_t(pyro::dji_motor_tx_frame_t::id_4, pyro::can_hub_t::can1);

        wheel_1_spd_pid = new pyro::pid_ctrl_t(1.0f, 0.1f, 0.0f);
        wheel_1_spd_pid->set_output_limits(20.0f);

        wheel_2_spd_pid = new pyro::pid_ctrl_t(1.0f, 0.1f, 0.0f);
        wheel_2_spd_pid->set_output_limits(20.0f);

        wheel_3_spd_pid = new pyro::pid_ctrl_t(1.0f, 0.1f, 0.0f);
        wheel_3_spd_pid->set_output_limits(20.0f);

        wheel_4_spd_pid = new pyro::pid_ctrl_t(1.0f, 0.1f, 0.0f);
        wheel_4_spd_pid->set_output_limits(20.0f);

        wheel_drv1 = new pyro::wheel_drv_t(wheel_motor1, wheel_1_spd_pid, 0.05f);
        wheel_drv2 = new pyro::wheel_drv_t(wheel_motor2, wheel_2_spd_pid, 0.05f);
        wheel_drv3 = new pyro::wheel_drv_t(wheel_motor3, wheel_3_spd_pid, 0.05f);
        wheel_drv4 = new pyro::wheel_drv_t(wheel_motor4, wheel_4_spd_pid, 0.05f);

        chassis_drv = new pyro::macunum_chassis_drv_t(wheel_drv1, wheel_drv2, wheel_drv3, wheel_drv4,pyro::macunum_chassis_drv_t::TYPE_X,.1,0.1);

        
        vTaskDelay(2000);
        
       
        vTaskDelay(1);
        for(;;)
        {

            chassis_drv->update();

            chassis_drv->set_target(vx_rc,vy_rc,wz_rc);
            

            if(mode == ZERO_FORCE)
            {
                chassis_drv->zero_force();
            }
            else
            {
               chassis_drv->control(0.001f);
            }

            
            vTaskDelay(1);
        }
    
    
}

}

#endif