#ifndef __PYRO__POWER_CONTROL_DRV_H__
#define __PYRO__POWER_CONTROL_DRV_H__

#include <vector>

namespace pyro
{


class power_control_drv_t
{
public:
    struct motor_coefficient_t
    {
        float k1;
        float k2;
        float k3;
        float k4;
    };

    struct wheel_data_t
    {
        float torque_cmd;       // 输入：当前扭矩指令
        float last_torque;
        float gyro;             // 输入：电机角速度
        float power_predict;    // 输入：预测功率
        float restricted_torque;// 输出：限制后扭矩
    };
    
    power_control_drv_t(const power_control_drv_t&) = delete;
    power_control_drv_t& operator=(const power_control_drv_t&) = delete;

    static power_control_drv_t& get_instance(int wheel_num)
    {
        static power_control_drv_t instance(wheel_num);
        return instance;
    }

    static power_control_drv_t& get_instance()
    {
        return get_instance(0);
    }

    float restrict_torque;
    void set_motor_coefficient(int wheel_index, const motor_coefficient_t& coefficient);    
    // void set_coefficient(float coefficient_k1, float coefficient_k2, float coefficient_k3, float coefficient_k4);
    float motor_power_predict(int wheel_index, float tau, float gyro) const;
    // void motor_power_predict(float tau, float gyro);
    // void motor_power_restrict_torque(float origin_torque,float gyro, float restricted_power);
    float motor_power_restrict_torque(int wheel_index, float origin_torque, float gyro, float restricted_power) const;
    void calculate_restricted_torques(
        wheel_data_t* wheel_data,
        int wheel_num,
        float power_limit
    ) const;
    void calculate_restricted_torques(
        wheel_data_t* wheel_data,
        int wheel_num,
        float power_limit,
        const float* power_ratios
    ) const;

private:
    explicit power_control_drv_t(int wheel_num);
    ~power_control_drv_t()
    {
    }

    std::vector<motor_coefficient_t> _motor_coefficients;
};

}

#endif

