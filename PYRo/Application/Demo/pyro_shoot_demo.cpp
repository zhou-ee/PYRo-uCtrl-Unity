#include "pyro_core_config.h"
#if SHOOT_DEMO_EN
#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"
#include "pyro_shoot_17mm_control.h"

#define FRIC_RADIUS 0.03f
#define STEP (pyro::PI/4)

#ifdef __cplusplus

extern "C"
{
    pyro::dji_m3508_motor_drv_t *m3508_drv_1;
    pyro::dji_m3508_motor_drv_t *m3508_drv_2;

    pyro::dji_m2006_motor_drv_t *m2006_drv;

    pyro::fric_drv_t *fric_drv_1;
    pyro::fric_drv_t *fric_drv_2;

    pyro::trigger_drv_t *trigger_drv;

    pyro::pid_ctrl_t *fric1_speed_pid;
    pyro::pid_ctrl_t *fric2_speed_pid;
    pyro::pid_ctrl_t *trigger_speed_pid;
    pyro::pid_ctrl_t *trigger_positon_pid;

    pyro::shoot_17mm_control_t *shoot_drv;

    void pyro_shoot_demo(void *arg)
    { 
        fric1_speed_pid = new pyro::pid_ctrl_t(5.0f, 0.0f, 0.00f);
        fric2_speed_pid = new pyro::pid_ctrl_t(5.0f, 0.0f, 0.00f);
        trigger_speed_pid = new pyro::pid_ctrl_t(2.0f, 0.1f, 0.00f);
        trigger_positon_pid = new pyro::pid_ctrl_t(2.0f, 0.01f, 0.00f);

        fric1_speed_pid->set_output_limits(15.0f);
        fric2_speed_pid->set_output_limits(15.0f);
        trigger_speed_pid->set_output_limits(15.0f);
        trigger_positon_pid->set_output_limits(15.0f);


        m3508_drv_1 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can2);
        m3508_drv_2 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_2, pyro::can_hub_t::can2);
        m2006_drv = new pyro::dji_m2006_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can2);

        fric_drv_1 = new pyro::fric_drv_t(
            m3508_drv_1, 
            *fric1_speed_pid,
            FRIC_RADIUS,
            pyro::fric_drv_t::CLOCKWISE);

        fric_drv_2 = new pyro::fric_drv_t(
            m3508_drv_2, 
            *fric2_speed_pid,
            FRIC_RADIUS,
            pyro::fric_drv_t::COUNTERCLOCKWISE);

        trigger_drv = new pyro::trigger_drv_t(
            m2006_drv,
            *trigger_speed_pid,
            *trigger_positon_pid,
            STEP);

        fric_drv_1->set_dt(0.001f);
        fric_drv_2->set_dt(0.001f);

        trigger_drv->set_dt(0.001f);
        trigger_drv->set_gear_ratio(36.0f);

        shoot_drv = new pyro::shoot_17mm_control_t(
            fric_drv_1,
            fric_drv_2,
            trigger_drv);

        shoot_drv->set_continuous_mode_delay(50);
        shoot_drv->set_fric_speed(23.0f);
        shoot_drv->set_trigger_rotate(10.0f);

        while (true)
        {
            // shoot_drv->update_feedback();
            // shoot_drv->_trigger_drv->set_radian(shoot_drv->_trigger_drv->get_radian() - STEP);
            // shoot_drv->_trigger_drv->set_rotate(10.0f);
            // shoot_drv->_trigger_drv->control();
            // shoot_drv->set_control();
            shoot_drv->control();

            vTaskDelay(1);
        }
    }



}

#endif
#endif
