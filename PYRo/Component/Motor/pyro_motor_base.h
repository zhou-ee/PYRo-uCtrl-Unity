#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#include "cmsis_os.h"
#include "main.h"
#include "pyro_can_drv.h"
#include "pyro_core_def.h"
#include <cstdint>

#include <memory>
#include <vector>



namespace pyro
{
class motor_base_t
{
  public:
    motor_base_t(can_hub_t::which_can which);
    // ~motor_base_t(void);//先不实现

    virtual pyro::status_t init(void)      = 0;
    virtual pyro::status_t enable()        = 0;
    virtual pyro::status_t disable()       = 0;

    virtual bool update_feedback(void)     = 0;
    virtual bool send_torque(float torque) = 0;

    int8_t get_temperature(void);
    float get_current_position(void);
    float get_current_rotate(void);
    float get_current_torque(void);

    bool is_enable(void);

  protected:
    // uint32_t _id;
    can_hub_t::which_can _which_can;
    can_drv_t *_can_drv;

    bool _enable;
    int8_t _temperature;

    float _current_position;
    float _current_rotate;
    float _current_torque;

    pyro::can_msg_buffer_t *_feedback_msg;
};
}; // namespace pyro

#endif