#include "pyro_power_control_drv.h"
#include <math.h>

namespace pyro
{
power_control_drv_t::power_control_drv_t()
{

}

void power_control_drv_t::set_coefficient(float coefficient_k1, float coefficient_k2, float coefficient_k3, float coefficient_k4)
{
    _coefficient_k1 = coefficient_k1;
    _coefficient_k2 = coefficient_k2;
    _coefficient_k3 = coefficient_k3;
    _coefficient_k4 = coefficient_k4;
}

void power_control_drv_t::motor_power_predict(float tau, float gyro)
{
    power_predict = _coefficient_k1*tau*gyro + _coefficient_k2*fabs(gyro) + _coefficient_k3*tau*tau + _coefficient_k4;
}

void power_control_drv_t::motor_power_restrict_torque(float origin_torque,float gyro, float restricted_power)
{
    float a = _coefficient_k3;
    float b = _coefficient_k1 * gyro;
    float c = _coefficient_k2 * fabs(gyro) + _coefficient_k4 - restricted_power;

    float delta = b*b - 4*a*c;
    if(delta <= 0)
    {
        restrict_torque = -b/(2*a);
        return;
    }
    else
    {
        if(origin_torque>0)
		{
			restrict_torque=(-b+sqrtf(delta))/(2*a);
		}
		else if(origin_torque<0)
		{
			restrict_torque=(-b-sqrtf(delta))/(2*a);
		}
		else
		{
			_coefficient_k1=0;
		}
		return;
    }
}





}
