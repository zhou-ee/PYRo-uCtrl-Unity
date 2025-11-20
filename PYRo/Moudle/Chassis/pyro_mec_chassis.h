#ifndef __PYRO_PYRO_MEC_CHASSIS_H__
#define __PYRO_PYRO_MEC_CHASSIS_H__

#include "pyro_chassis_base.h"
#include "pyro_motor_base.h"

namespace pyro
{
class mec_chassis_t final : public chassis_base_t
{
  public:
    struct cmd_mec_t final : cmd_base_t
    {
        cmd_mec_t() : cmd_base_t(type_t::MECANUM)
        {
        }
        // Additional mecanum-specific command parameters can be added here
    };
    explicit mec_chassis_t(motor_base_t ** wheel_motor_array):chassis_base_t(type_t::MECANUM)
    {
        _wheel_motor [0] = wheel_motor_array[0];
        _wheel_motor [1] = wheel_motor_array[1];
        _wheel_motor [2] = wheel_motor_array[2];
        _wheel_motor [3] = wheel_motor_array[3];
    }
    ~mec_chassis_t() override;

    void set_command(const cmd_base_t &cmd) override;
    void update_feedback() override;
    void kinematics_solve() override;
    void chassis_control() override;
    void power_control() override;
    void send_motor_command() override;

  private:
    motor_base_t *_wheel_motor[4]{}; // FL, FR, BL, BR
    // Add private members and methods as needed
};
} // namespace pyro
#endif
