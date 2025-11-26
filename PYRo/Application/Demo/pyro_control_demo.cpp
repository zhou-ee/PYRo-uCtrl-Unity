#include "pyro_core_config.h"
#if CONTROL_DEMO_EN
#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"
#include "pyro_chassis_drv.h"
#include "pyro_rc_hub.h"
#include "pyro_yaw_drv.h"
#include "pyro_rw_lock.h"
#include "pyro_rc_hub.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_powermeter.h"

#ifdef __cplusplus

pyro::rc_drv_t *dr16_drv;

typedef struct
{
    float err;
    float kp;
    float ki;
    float kd;
} debug_t;

debug_t debug;

extern "C"
{
    pyro::dji_m3508_motor_drv_t *m3508_drv_1;
    pyro::dji_m3508_motor_drv_t *m3508_drv_2;
    pyro::dji_m3508_motor_drv_t *m3508_drv_3;
    pyro::dji_m3508_motor_drv_t *m3508_drv_4;

    pyro::dji_gm_6020_motor_drv_t *gm6020_drv_1;
    pyro::dji_gm_6020_motor_drv_t *gm6020_drv_2;
    pyro::dji_gm_6020_motor_drv_t *gm6020_drv_3;
    pyro::dji_gm_6020_motor_drv_t *gm6020_drv_4;
    pyro::dji_gm_6020_motor_drv_t *gm6020_drv_5;

    pyro::wheel_drv_t *wheel_drv_1;
    pyro::wheel_drv_t *wheel_drv_2;
    pyro::wheel_drv_t *wheel_drv_3;
    pyro::wheel_drv_t *wheel_drv_4;

    pyro::pid_t *speed_pid_1;
    pyro::pid_t *speed_pid_2;
    pyro::pid_t *speed_pid_3;
    pyro::pid_t *speed_pid_4;

    pyro::pid_t *follow_yaw_pid;

    pyro::steering_wheel_drv_t *steering_wheel_drv_1;
    pyro::steering_wheel_drv_t *steering_wheel_drv_2;
    pyro::steering_wheel_drv_t *steering_wheel_drv_3;
    pyro::steering_wheel_drv_t *steering_wheel_drv_4;

    pyro::yaw_drv_t *yaw_drv_1;

    pyro::chassis_drv_t *chassis_drv;

    pyro::pid_t *rudder_rotate_pid_1;
    pyro::pid_t *rudder_position_pid_1;
    pyro::pid_t *rudder_rotate_pid_2;
    pyro::pid_t *rudder_rotate_pid_3;
    pyro::pid_t *rudder_rotate_pid_4;
    pyro::pid_t *rudder_position_pid_2;
    pyro::pid_t *rudder_position_pid_3;
    pyro::pid_t *rudder_position_pid_4;

    pyro::pid_t *yaw_rotate_pid_1;
    pyro::pid_t *yaw_position_pid_1;

    pyro::powermeter_drv_t *power_meter;

    extern IMU_obj Imu;

    pyro::powermeter_data power_data;

    void pyro_control_demo(void *arg)
    {
        pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart5)->enable_rx_dma();
        dr16_drv = pyro::rc_hub_t::get_instance(
            pyro::rc_hub_t::DR16);
        dr16_drv->init();
        dr16_drv->enable();

        /* ------ PID 初始化 -------------------------------------------------*/

        // 1. 系数
        speed_pid_1 = new pyro::pid_t(24.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        speed_pid_2 = new pyro::pid_t(24.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        speed_pid_3 = new pyro::pid_t(24.0f, 0.1f, 0.00f, 1.00f, 20.0f);
        speed_pid_4 = new pyro::pid_t(24.0f, 0.1f, 0.00f, 1.00f, 20.0f);


        rudder_position_pid_1 = new pyro::pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
        rudder_position_pid_2 = new pyro::pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
        rudder_position_pid_3 = new pyro::pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);
        rudder_position_pid_4 = new pyro::pid_t(15.0f, 0.0f, 0.00f, 0.0f, 10.0f);

        rudder_rotate_pid_1 = new pyro::pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
        rudder_rotate_pid_2 = new pyro::pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
        rudder_rotate_pid_3 = new pyro::pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
        rudder_rotate_pid_4 = new pyro::pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);

        follow_yaw_pid = new pyro::pid_t(3.6f, 0.01f, 0.003f, 0.1f, 5.0f);

        yaw_position_pid_1 = new pyro::pid_t(20.0f, 0.2f, 0.02f, 0.5f, 10.0f, 15, 150, 4);
        yaw_rotate_pid_1 = new pyro::pid_t(0.3f, 0.003f, 0.0003f, 0.1f, 3.0f, 15, 150, 4);

        

        m3508_drv_1 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can1);
        m3508_drv_2 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_2, pyro::can_hub_t::can2);
        m3508_drv_3 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can2);
        m3508_drv_4 = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_4, pyro::can_hub_t::can1);

        gm6020_drv_1 = new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can1);
        gm6020_drv_2 = new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_2, pyro::can_hub_t::can2);
        gm6020_drv_3 = new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_3, pyro::can_hub_t::can2);
        gm6020_drv_4 = new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_4, pyro::can_hub_t::can1);
        gm6020_drv_5 = new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_5, pyro::can_hub_t::can2);

        wheel_drv_1 = new pyro::wheel_drv_t(
            m3508_drv_1,
            *speed_pid_1,
            0.0685f);

        wheel_drv_2 = new pyro::wheel_drv_t(
            m3508_drv_2,
            *speed_pid_2,
            0.06f);

        wheel_drv_3 = new pyro::wheel_drv_t(
           m3508_drv_3,
            *speed_pid_3,
            0.06f);
        
        wheel_drv_4 = new pyro::wheel_drv_t(
            m3508_drv_4,
            *speed_pid_4,
            0.0685f);

        wheel_drv_1->set_gear_ratio(19.0f);
        wheel_drv_2->set_gear_ratio(19.0f);
        wheel_drv_3->set_gear_ratio(19.0f);
        wheel_drv_4->set_gear_ratio(19.0f);

        steering_wheel_drv_1 = new pyro::steering_wheel_drv_t(
            wheel_drv_1,
            gm6020_drv_1,
            *rudder_rotate_pid_1,
            *rudder_position_pid_1);

        steering_wheel_drv_2 = new pyro::steering_wheel_drv_t(
            wheel_drv_2,
            gm6020_drv_2,
            *rudder_rotate_pid_2,
            *rudder_position_pid_2);

        steering_wheel_drv_3 = new pyro::steering_wheel_drv_t(
            wheel_drv_3,
            gm6020_drv_3,
            *rudder_rotate_pid_3,
            *rudder_position_pid_3);

        steering_wheel_drv_4 = new pyro::steering_wheel_drv_t(
            wheel_drv_4,
            gm6020_drv_4,
            *rudder_rotate_pid_4,
            *rudder_position_pid_4);

        yaw_drv_1 = new pyro::yaw_drv_t(
            gm6020_drv_5,
            *yaw_rotate_pid_1,
            *yaw_position_pid_1,
            &Imu);

        steering_wheel_drv_1->set_offset_radian(1.76254392f);
        steering_wheel_drv_2->set_offset_radian(-1.817f);
        steering_wheel_drv_3->set_offset_radian(0.753184557f);
        steering_wheel_drv_4->set_offset_radian(2.3554275f);
        yaw_drv_1           ->set_offset_radian(-2.1675148f);

        chassis_drv = new pyro::chassis_drv_t(
            steering_wheel_drv_1,
            steering_wheel_drv_2,
            steering_wheel_drv_3,
            steering_wheel_drv_4,
            follow_yaw_pid,
            4);

        pyro::power_control_drv_t& power_controller = pyro::power_control_drv_t::get_instance();
        pyro::power_control_drv_t::motor_coefficient_t coeff1;
        coeff1.k1 = 0.0118f;
        coeff1.k2 = 0.0163f;
        coeff1.k3 = 0.1945f;
        coeff1.k4 = 0.6456f;
        power_controller.set_motor_coefficient(1, coeff1);

        pyro::power_control_drv_t::motor_coefficient_t coeff2;
        coeff2.k1 = 0.0123f;
        coeff2.k2 = 0.0218f;
        coeff2.k3 = 0.1597f;
        coeff2.k4 = 0.7248f;
        power_controller.set_motor_coefficient(2, coeff2);

        pyro::power_control_drv_t::motor_coefficient_t coeff3;
        coeff3.k1 = 0.0127f;
        coeff3.k2 = 0.0225f;
        coeff3.k3 = 0.1455f;
        coeff3.k4 = 0.2370f;
        power_controller.set_motor_coefficient(3, coeff3);

        pyro::power_control_drv_t::motor_coefficient_t coeff4;
        coeff4.k1 = 0.0099f;
        coeff4.k2 = 0.0094f;
        coeff4.k3 = 0.1969f;
        coeff4.k4 = 0.7185f;
        power_controller.set_motor_coefficient(4, coeff4);

        power_meter = new pyro::powermeter_drv_t(0x212 ,pyro::can_hub_t::can3);

        power_meter->init();

        while (true)
        {
            // 1. 更新电机数据
            chassis_drv->update_feedback();

            // 2. 更新Yaw数据
            yaw_drv_1->update_feedback();
            yaw_drv_1->set_control();
            yaw_drv_1->control();

            power_meter->get_data(power_data);

            chassis_drv->chassis_control(-yaw_drv_1->get_radian());
            // chassis_drv->chassis_control(0);
            // chassis_drv->chassis_control(-0.0322136879f);



            vTaskDelay(1);
        }
    }
}
#endif
#endif
