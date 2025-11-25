#include "pyro_power_control_drv.h"
#include <math.h>

namespace pyro
{
// power_control_drv_t::power_control_drv_t(int wheel_num)
// {
//     if (wheel_num <= 0) 
//     {
//     }
//     _motor_coefficients.resize(wheel_num, {0.0f, 0.0f, 0.0f, 0.0f});
// }

// void power_control_drv_t::set_coefficient(float coefficient_k1, float coefficient_k2, float coefficient_k3, float coefficient_k4)
// {
//     _coefficient_k1 = coefficient_k1;
//     _coefficient_k2 = coefficient_k2;
//     _coefficient_k3 = coefficient_k3;


//     _coefficient_k4 = coefficient_k4;
// }

void power_control_drv_t::set_motor_coefficient(int wheel_index, const motor_coefficient_t& coefficient)
{
    int zero_based_index = wheel_index - 1;
    if (zero_based_index >= 0 && zero_based_index < static_cast<int>(_motor_coefficients.size())) 
    {
            _motor_coefficients[zero_based_index] = coefficient;
    }
}

float power_control_drv_t::motor_power_predict(int wheel_index, float tau, float gyro) const
{
    if (wheel_index >= 0 && wheel_index < static_cast<int>(_motor_coefficients.size())) 
    {
        const motor_coefficient_t& coefficient = _motor_coefficients[wheel_index];
        return coefficient.k1*tau*gyro + coefficient.k2*fabs(gyro) + coefficient.k3*tau*tau + coefficient.k4;
    }
    else
    {
        return 0.0f;
    }
}


// void power_control_drv_t::motor_power_predict(float tau, float gyro)
// {
//     power_predict = _coefficient_k1*tau*gyro + _coefficient_k2*fabs(gyro) + _coefficient_k3*tau*tau + _coefficient_k4;
//     // power_predict = _coefficient_k1*fabs(tau)*fabs(gyro) + _coefficient_k2*fabs(gyro) + _coefficient_k3*tau*tau + _coefficient_k4;
// }

// void power_control_drv_t::motor_power_restrict_torque(float origin_torque,float gyro, float restricted_power)
// {
//     float a = _coefficient_k3;
//     float b = _coefficient_k1 * gyro;
//     float c = _coefficient_k2 * fabs(gyro) + _coefficient_k4 - restricted_power;

//     float delta = b*b - 4*a*c;
//     if(delta <= 0)
//     {
//         restrict_torque = -b/(2*a);
//         return;
//     }
//     else
//     {
//         if(origin_torque > 0)
// 		{
// 			restrict_torque=(-b+sqrtf(delta))/(2*a);
// 		}
// 		else if(origin_torque < 0)
// 		{
// 			restrict_torque = (-b-sqrtf(delta))/(2*a);
// 		}
// 		else
// 		{
// 			restrict_torque = 0;
// 		}
// 		return;
//     }
// }

float power_control_drv_t::motor_power_restrict_torque(int wheel_index, float origin_torque, float gyro, float restricted_power) const
{
    const motor_coefficient_t& coefficient = _motor_coefficients[wheel_index];
    float a = coefficient.k3;
    float b = coefficient.k1 * gyro;
    float c = coefficient.k2 * fabs(gyro) + coefficient.k4 - restricted_power;
    float delta = b * b - 4 * a * c;
    if (delta <= 0) 
    {
        return -b / (2 * a);
    } 
    else 
    {
        float sqrt_delta = std::sqrt(delta);
        if (origin_torque > 0) 
        {
            return (-b + sqrt_delta) / (2 * a);
        } 
        else if (origin_torque < 0) 
        {
            return (-b - sqrt_delta) / (2 * a);
        } 
        else 
        {
            return 0.0f;
        }
    }
}

void power_control_drv_t::calculate_restricted_torques(
    wheel_data_t* wheel_data,
    int wheel_num,
    float power_limit
) const
{ 
    calculate_restricted_torques(wheel_data, wheel_num, power_limit, nullptr);
}

void power_control_drv_t::calculate_restricted_torques(
    wheel_data_t* wheel_data,
    int wheel_num,
    float power_limit,
    const float* power_ratios
) const
{
    if (wheel_num <= 0 || wheel_data == nullptr || power_limit <= 0) {
        return;
    }

    std::vector<float> ratios(wheel_num);
    if (nullptr == power_ratios) 
    {
        float avg_ratio = 1.0f / wheel_num;
        for (int i = 0; i < wheel_num; i++) 
        {
            ratios[i] = avg_ratio;
        }
    }
    else
    {
        for (int i = 0; i < wheel_num; i++) 
        {
            ratios[i] = power_ratios[i];
        }
    }

    float total_power = 0.0f;
    for (int i = 0; i < wheel_num; i++) 
    {
        total_power += wheel_data[i].power_predict;
    }

    float alpha = 1.0f;
    const float POWER_THRESHOLD = power_limit * 1.1f;
    if (total_power > POWER_THRESHOLD) 
    {
        alpha = 0.8f;
    } 
    else if (total_power > power_limit) 
    {
        alpha = 0.10f;
    }

    if (total_power > power_limit) 
    {
        for (int i = 0; i < wheel_num; i++) 
        {
            float wheel_power_limit = power_limit * ratios[i];
            float restricted_torque = motor_power_restrict_torque(
                i,                               
                wheel_data[i].torque_cmd,      
                wheel_data[i].gyro,            
                wheel_power_limit
            );

            wheel_data[i].restricted_torque = 
                alpha * restricted_torque + 
                (1 - alpha) * wheel_data[i].last_torque;
        }
    }
    else 
    {
        for (int i = 0; i < wheel_num; i++) 
        {
            wheel_data[i].restricted_torque = wheel_data[i].torque_cmd;
        }
    }

    for(int i = 0; i < wheel_num; i++)
    {
        wheel_data[i].last_torque = wheel_data[i].restricted_torque;
    }
}

power_control_drv_t::power_control_drv_t(int wheel_num)
{
    _motor_coefficients.resize(wheel_num);
}




}
