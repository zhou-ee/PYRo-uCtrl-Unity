#include "pyro_core_config.h"
#if GIMBAL_DEMO_EN
#include "cmsis_os.h"
#include "fdcan.h"
#include "pyro_can_drv.h"
#include "pyro_gimbal_drv.h"


#ifdef __cplusplus
extern "C"
{
    pyro::dm_motor_drv_t *dm_drv_pitch;
    pyro::dm_motor_drv_t *dm_drv_roll;
    pyro::dji_gm_6020_motor_drv_t *gm6020_drv;

    pyro::pitch_drv_t *pitch_drv;
    pyro::roll_drv_t *roll_drv;
    pyro::yaw_drv_t *yaw_drv;

    pyro::pid_ctrl_t *pitch_speed_pid;
    pyro::pid_ctrl_t *pitch_position_pid;
    pyro::pid_ctrl_t *roll_speed_pid;
    pyro::pid_ctrl_t *roll_position_pid;
    pyro::pid_ctrl_t *yaw_speed_pid;
    pyro::pid_ctrl_t *yaw_position_pid;

    pyro::gimbal_drv_t *gimbal_drv;

    extern IMU_obj Imu;
    void pyro_gimbal_demo(void *arg)
    {
        pitch_speed_pid = new pyro::pid_ctrl_t(0.6f, 0.0f, 0.0f);
        pitch_position_pid = new pyro::pid_ctrl_t(10.0f, 0.0f, 0.0f);
        roll_speed_pid = new pyro::pid_ctrl_t(0.4f, 0.0f, 0.0f);
        roll_position_pid = new pyro::pid_ctrl_t(10.0f, 0.0f, 0.0f);
        yaw_speed_pid = new pyro::pid_ctrl_t(0.4f, 0.0f, 0.0f);
        yaw_position_pid = new pyro::pid_ctrl_t(15.0f, 0.0f, 0.0f);

        pitch_speed_pid->set_output_limits(10.0f);
        pitch_position_pid->set_output_limits(10.0f);
        roll_speed_pid->set_output_limits(10.0f);
        roll_position_pid->set_output_limits(10.0f);
        yaw_speed_pid->     set_output_limits(10.0f);
        yaw_position_pid->  set_output_limits(10.0f);

        dm_drv_pitch = new pyro::dm_motor_drv_t(0x30, 0x40, pyro::can_hub_t::can2);
        dm_drv_pitch->set_position_range(-pyro::PI, pyro::PI);
        dm_drv_pitch->set_rotate_range(-20, 20);
        dm_drv_pitch->set_torque_range(-10, 10);
        dm_drv_pitch->enable();

        dm_drv_roll = new pyro::dm_motor_drv_t(0x01, 0x00, pyro::can_hub_t::can1);
        dm_drv_roll->set_position_range(-pyro::PI, pyro::PI);
        dm_drv_roll->set_rotate_range(-20, 20);
        dm_drv_roll->set_torque_range(-10, 10);
        dm_drv_roll->enable();

        gm6020_drv = new pyro::dji_gm_6020_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can1);

        pitch_drv = new pyro::pitch_drv_t(
            dm_drv_pitch, 
            *pitch_speed_pid, 
            *pitch_position_pid);

        roll_drv = new pyro::roll_drv_t(
            dm_drv_roll, 
            *roll_speed_pid, 
            *roll_position_pid);

        yaw_drv = new pyro::yaw_drv_t(
            gm6020_drv,
            *yaw_speed_pid,
            *yaw_position_pid);

        gimbal_drv->set_dt(0.001);

        gimbal_drv = new pyro::gimbal_drv_t(pitch_drv, roll_drv, yaw_drv, &Imu);
        gimbal_drv->update_feedback();
        gimbal_drv->zero_force();

        while (true)
        {
            gimbal_drv->update_feedback();
            gimbal_drv->set_control();
            gimbal_drv->control();
            vTaskDelay(1);
        }
    }



}

#endif
#endif


//6020 CAN1 ID 0x01 -0.9(left) to -PI , PI to 2.24(right)
//pitch CAN2 0x30 0x40 0.15(highest) to -0.48(lowest)
//ROLL CAN1 0x01 0x00 0.3(left) to -0.34(right)