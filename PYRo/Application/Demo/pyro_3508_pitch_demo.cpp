/*
 * @Author: lucky zhou_20006ee@163.com
 * @Date: 2025-11-12 19:43:45
 * @LastEditors: lucky zhou_20006ee@163.com
 * @LastEditTime: 2025-11-14 02:59:02
 * @FilePath: \PYRo-uCtrl-Unity\PYRo\Application\Demo\pyro_3508_pitch_demo.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置
 * 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "IMU_Base.h"
#include "cmsis_os.h"
#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_pitch_drv.h"
#include "pyro_rc_hub.h"
#include "task.h"

extern IMU_obj Imu;

#ifdef __cplusplus

extern "C"
{
    pyro::pid_t *spd_pid, *pos_pid;
    pyro::motor_base_t *pitch_motor;
    pyro::pitch_drv_t *pitch_drv;
    uint8_t zero_flag = 1;

    void pyro_3508_pitch_demo(void *arg)
    {
        vTaskDelay(500);
        pos_pid     = new pyro::pid_t(20.0f, 1.0f, 0.5f, 1.0f, 8.0f,
                                      0.05305164769729844f, 0.01326291192432461f, 4);
        spd_pid     = new pyro::pid_t(50.0f, 2.2f, 1.0f, 2.0f, 18.0f,
                                      0.05305164769729844f, 0.01326291192432461f, 4);

        pitch_motor = new pyro::dji_m3508_motor_drv_t(
            pyro::dji_motor_tx_frame_t::id_1, pyro::can_hub_t::can2);

        pitch_drv = new pyro::pitch_drv_t(pitch_motor, *spd_pid, *pos_pid);

        pyro::rc_hub_t::get_instance(pyro::rc_hub_t::DR16)
            ->config_rc_cmd(
                [](void const *rc_ctrl) -> void
                {
                    static auto *p_ctrl =
                        static_cast<pyro::dr16_drv_t::dr16_ctrl_t const *>(
                            rc_ctrl);
                    if (p_ctrl->rc.s[pyro::dr16_drv_t::DR16_SW_RIGHT].state !=
                        pyro::dr16_drv_t::DR16_SW_MID)
                    {
                        zero_flag = true;
                    }
                    else if (zero_flag)
                    {
                        zero_flag = false;
                    }
                    if (!zero_flag)
                    {
                        pitch_drv->update_target_radian(
                            p_ctrl->rc.ch[pyro::dr16_drv_t::DR16_CH_LEFT_Y] *
                            0.01f);
                    }
                });
        while (true)
        {
            pitch_motor->update_feedback();
            pitch_drv->update_feedback(Imu.Get_Pitch(&Imu), Imu.gyro[1]);
            if (zero_flag)
            {
                pitch_drv->_target_radian = pitch_drv->_current_radian;
                pitch_drv->zero_force();
            }
            else
            {
                pitch_drv->calculate_torque();
            } 
            pitch_drv->output_torque();
            vTaskDelay(1);
        }
    }
}

#endif
