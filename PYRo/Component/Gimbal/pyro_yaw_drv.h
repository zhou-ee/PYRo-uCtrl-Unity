#ifndef __PYRO_YAW_DRV_H__
#define __PYRO_YAW_DRV_H__

#include "pyro_motor_base.h"
#include "pyro_pid_ctrl.h"
#include "IMU_Base.h"
#include "pyro_rc_hub.h"

namespace pyro
{
class yaw_drv_t
{
    public:
      enum total_mode_t
      {
        ZERO_FORCE         = 0x00,
        RC_CONTROL         = 0x01,
        AUTO_AIM_CONTROL   = 0x02
      };
      yaw_drv_t(motor_base_t *motor_base,
                const pid_ctrl_t &rotate_pid,
                const pid_ctrl_t &position_pid,
                IMU_obj *imu);
      ~yaw_drv_t()
      {
      }

      void set_offset_radian(float offset_radian);
      void set_radian(float target_radian);
      void zero_force();
      void dr16_cmd(void const *rc_ctrl);
      void vt03_cmd(void const *rc_ctrl);
      void update_feedback();
      float get_yaw_motor_err();
      float get_target_radian();
      float get_radian();
      void set_control();
      void control();
      float _offset_radian;
      motor_base_t *motor_base;

    private:
      pid_ctrl_t _yaw_rotate_pid;
      pid_ctrl_t _yaw_position_pid;
      IMU_obj *_imu{};
      total_mode_t _total_mode;
      float _current_rotate;
      float _current_yaw_imu_radian;
      float _target_yaw_imu_radian;
      float _yaw_motor_err;
      float _target_radian;
      float _current_radian;
      float _IMU_zero_point;
      float _yaw_cmd_torque;
};


}

#endif

