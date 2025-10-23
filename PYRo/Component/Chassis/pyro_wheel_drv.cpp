#include "pyro_wheel_drv.h"

namespace pyro
{
void wheel_drv_t::update()
{
    velocity_controller_t::update();
    _feedback_spd = _feedback_spd * _radius;
}
};