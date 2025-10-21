#include "pyro_core_config.h"


#if ENGINEER_ARM_DEMO_EN

#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"

#include "pyro_position_controller.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_pid_ctrl.h"

#include "pyro_dr16_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_uart_drv.h"

extern "C"
{

typedef enum 
{
    ZERO_FORCE,
    RC_CONTROL
}
mode_t;

mode_t mode = ZERO_FORCE;
float d_axis1_angle = 0.0f;
float d_axis2_angle = 0.0f;
float d_axis3_angle = 0.0f;

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

    d_axis1_angle = static_cast<float>(p_ctrl->rc.ch[0]) / 660.0f * 0.001;
    d_axis2_angle = static_cast<float>(p_ctrl->rc.ch[1]) / 660.0f * 0.001;
    d_axis3_angle = static_cast<float>(p_ctrl->rc.ch[2]) / 660.0f * 0.01;
}



pyro::can_drv_t *can1_drv;
pyro::can_drv_t *can2_drv;
pyro::can_drv_t *can3_drv;


pyro::position_controller_t *axis1_ctrl;
pyro::dm_motor_drv_t *axis1_motor;
pyro::pid_ctrl_t *axis1_spd_pid,*axis1_pos_pid;
float axis1_angle = 0.0f;

pyro::position_controller_t *axis2_ctrl;
pyro::dm_motor_drv_t *axis2_motor;
pyro::pid_ctrl_t *axis2_spd_pid,*axis2_pos_pid;
float axis2_angle = 0.0f;

pyro::position_controller_t *axis3_ctrl;
pyro::dm_motor_drv_t *axis3_motor;
pyro::pid_ctrl_t *axis3_spd_pid,*axis3_pos_pid;
float axis3_angle = 0.0f;





void pyro_engineer_arm_demo(void *arg)
{
        pyro::get_uart5().enable_rx_dma();
        dr16_drv = new pyro::dr16_drv_t(&pyro::get_uart5());
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


        // axis1 init code
        axis1_motor = new pyro::dm_motor_drv_t(0x1, 0x0, pyro::can_hub_t::can1);
        axis1_motor->set_position_range(-pyro::PI, pyro::PI);
        axis1_motor->set_rotate_range(-45, 45); 
        axis1_motor->set_torque_range(-54, 54);

        axis1_spd_pid   = new pyro::pid_ctrl_t(5.0f, 0.1f, 0.0f);
        axis1_spd_pid->set_output_limits(54.0f);
        axis1_spd_pid->set_integral_limits(5.0f);

        axis1_pos_pid   = new pyro::pid_ctrl_t(3.2f, 0.0f, 0.0f);
        axis1_pos_pid->set_output_limits(45.0f);
        axis1_pos_pid->set_integral_limits(5.0f);

        axis1_ctrl  = new pyro::position_controller_t(
            axis1_motor, axis1_pos_pid, axis1_spd_pid);

        axis1_ctrl->enable_constraint();
        axis1_ctrl->set_upper_limit(2.8f);
        axis1_ctrl->set_lower_limit(0);

        //axis2 init code
        axis2_motor = new pyro::dm_motor_drv_t(0x3, 0x2, pyro::can_hub_t::can1);
        axis2_motor->set_position_range(-pyro::PI, pyro::PI);
        axis2_motor->set_rotate_range(-20, 20); 
        axis2_motor->set_torque_range(-10, 10);

        axis2_spd_pid   = new pyro::pid_ctrl_t(5.0f, 0.1f, 0.0f);
        axis2_spd_pid->set_output_limits(10.0f);
        axis2_spd_pid->set_integral_limits(5.0f);

        axis2_pos_pid   = new pyro::pid_ctrl_t(3.2f, 0.0f, 0.0f);
        axis2_pos_pid->set_output_limits(20.0f);
        axis2_pos_pid->set_integral_limits(5.0f);

        axis2_ctrl  = new pyro::position_controller_t(
            axis2_motor, axis2_pos_pid, axis2_spd_pid);

        axis2_ctrl->enable_constraint();
        axis2_ctrl->set_upper_limit(2.8f);
        axis2_ctrl->set_lower_limit(0);

        //axis3 init code
        axis3_motor = new pyro::dm_motor_drv_t(0x5, 0x4, pyro::can_hub_t::can1);
        axis3_motor->set_position_range(-pyro::PI, pyro::PI);
        axis3_motor->set_rotate_range(-20, 20); 
        axis3_motor->set_torque_range(-10, 10);

        axis3_spd_pid   = new pyro::pid_ctrl_t(1.0f, 0.1f, 0.0f);
        axis3_spd_pid->set_output_limits(10.0f);
        axis3_spd_pid->set_integral_limits(5.0f);

        axis3_pos_pid   = new pyro::pid_ctrl_t(1.6f, 0.0f, 0.0f);
        axis3_pos_pid->set_output_limits(20.0f);
        axis3_pos_pid->set_integral_limits(5.0f);

        axis3_ctrl  = new pyro::position_controller_t(
            axis3_motor, axis3_pos_pid, axis3_spd_pid);

        // axis3_ctrl->set_upper_limit(pyro::PI/2);
        // axis3_ctrl->set_lower_limit(-pyro::PI/2);

        // motor = new pyro::dm_motor_drv_t(0x5, 0x4, pyro::can_hub_t::can1);
        // motor->set_position_range(-pyro::PI, pyro::PI);
        // motor->set_rotate_range(-20, 20);
        // motor->set_torque_range(-10, 10);
        
        // spd_pid   = new pyro::pid_ctrl_t(1.0f, 0.1f, 0.0f);
        // spd_pid->set_output_limits(10.0f);
        // spd_pid->set_integral_limits(5.0f);

        // pos_pid   = new pyro::pid_ctrl_t(1.6f, 0.0f, 0.0f);
        // pos_pid->set_output_limits(20.0f);
        // pos_pid->set_integral_limits(5.0f);

        // ctrl  = new pyro::position_controller_t(
        //     motor, pos_pid, spd_pid);
        // ctrl->set_target(-1.0f);
        vTaskDelay(2000);
        axis1_motor->enable();
        vTaskDelay(1);
        axis1_motor->update_feedback();
        //axis1_angle = axis1_motor->get_current_position();
        axis2_motor->enable();
        vTaskDelay(1);
        axis2_motor->update_feedback();
       // axis2_angle = axis2_motor->get_current_position();
        axis3_motor->enable();
        vTaskDelay(1);
        axis3_motor->update_feedback();
        //axis3_angle = axis3_motor->get_current_position();
       
        vTaskDelay(1);
        for(;;)
        {

            
            axis1_angle+=d_axis1_angle;
            axis2_angle+=d_axis2_angle;
            axis3_angle+=d_axis3_angle;

            if(axis1_angle>pyro::PI)
                axis1_angle-=pyro::PI*2;
            else if(axis1_angle<-pyro::PI)
                axis1_angle+=pyro::PI*2;

            if(axis2_angle>pyro::PI)
                axis2_angle-=pyro::PI*2;
            else if(axis2_angle<-pyro::PI)
                axis2_angle+=pyro::PI*2;

            if(axis3_angle>pyro::PI)
                axis3_angle-=pyro::PI*2;
            else if(axis3_angle<-pyro::PI)  
                axis3_angle+=pyro::PI*2;

            axis1_ctrl->update();
            axis2_ctrl->update();
            axis3_ctrl->update();

            axis1_angle=axis1_ctrl->set_target(axis1_angle);
            axis2_angle=axis2_ctrl->set_target(axis2_angle);
            axis3_ctrl->set_target(axis3_angle);

            if(mode == ZERO_FORCE)
            {
                
            //    axis1_motor->send_torque(0.0f);
            //     axis3_motor->send_torque(0.0f);
            //     axis2_motor->send_torque(0.0f);
                axis3_ctrl->zero_force();
                axis1_ctrl->zero_force();
                axis2_ctrl->zero_force();
                 
            }
            else
            {
               
                
                axis2_ctrl->control(0.001f);
                axis3_ctrl->control(0.001f);
                axis1_ctrl->control(0.001f);
                // axis1_motor->send_torque(0.0f);
                // axis3_motor->send_torque(0.0f);
                // axis2_motor->send_torque(0.0f);
               
            }

            
            vTaskDelay(1);
        }
    
    
}

}

#endif