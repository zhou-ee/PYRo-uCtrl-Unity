#ifndef __PYRO__POWER_CONTROL_DRV_H__
#define __PYRO__POWER_CONTROL_DRV_H__

namespace pyro
{
class power_control_drv_t
{
public:
    power_control_drv_t();
    ~power_control_drv_t()
    {
    }

    float power_predict;
    float restrict_torque;
    void set_coefficient(float coefficient_k1, float coefficient_k2, float coefficient_k3, float coefficient_k4);
    void motor_power_predict(float tau, float gyro);
    void motor_power_restrict_torque(float origin_torque,float gyro, float restricted_power);

private:
    float _coefficient_k1;//tau*gyro
    float _coefficient_k2;//abs(gyro)
    float _coefficient_k3;//tau^2
    float _coefficient_k4;//1
};

}

#endif

