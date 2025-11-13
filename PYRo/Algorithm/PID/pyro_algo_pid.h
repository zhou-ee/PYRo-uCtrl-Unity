#ifndef __PYRO_ALGO_PID_H__
#define __PYRO_ALGO_PID_H__

#include "pyro_algo_ols.h"
#include <cstdint>

namespace pyro
{

class pid_t
{
public:
    /**
     * @brief PID 改进选项 (来自 controller.h)
     */
    enum Improvement : uint8_t
    {
        NONE = 0x00,                        // 0000 0000
        Integral_Limit = 0x01,              // 0000 0001
        Derivative_On_Measurement = 0x02,   // 0000 0010
        Trapezoid_Intergral = 0x04,         // 0000 0100
        Proportional_On_Measurement = 0x08, // 0000 1000 (未使用)
        OutputFilter = 0x10,                // 0001 0000
        ChangingIntegrationRate = 0x20,     // 0010 0000
        DerivativeFilter = 0x40,            // 0100 0000
        ErrorHandle = 0x80,                 // 1000 0000
    };

    /**
     * @brief 错误类型 (来自 controller.h)
     */
    enum class ErrorType : uint8_t
    {
        NONE = 0x00U,
        Motor_Blocked = 0x01U
    };

    /**
     * @brief 错误处理器 (来自 controller.h)
     */
    struct ErrorHandler
    {
        uint64_t ERRORCount;
        ErrorType ERRORType;
    };

    /**
     * @brief 用户回调函数指针类型
     * @param pid 指向当前PID实例的指针
     */
    using UserFunc_t = void (*)(pid_t *pid);

    // --- 构造函数 (按你的要求重载) ---

    /**
     * @brief 构造函数 1: 基本PID
     */
    pid_t(float kp, float ki, float kd,
        float integral_limit,
        float max_out,
        uint8_t improve = Integral_Limit);

    /**
     * @brief 构造函数 2: 带滤波和OLS的PID
     */
    pid_t(float kp, float ki, float kd,
        float integral_limit,
        float max_out,
        float output_lpf_rc,
        float derivative_lpf_rc,
        uint16_t ols_order,
        uint8_t improve = Integral_Limit | OutputFilter | DerivativeFilter);

    /**
     * @brief 构造函数 3: 完整版 (主构造函数)
     * (此构造函数是C语言 PID_Init 的直接替代)
     */
    pid_t(float max_out,
        float integral_limit,
        float deadband,
        float kp,
        float ki,
        float kd,
        float A, // 变速积分 CoefA
        float B, // 变速积分 CoefB
        float output_lpf_rc,
        float derivative_lpf_rc,
        uint16_t ols_order,
        uint8_t improve);

    /**
     * @brief 计算PID输出
     * @param measure 测量值
     * @param ref 期望值
     * @return PID输出
     */
    float calculate(float measure, float ref);

    /**
     * @brief 清除PID内部状态 (Iout, Dout, LastErr 等)
     */
    void clear();

    /**
     * @brief 设置新的PID增益
     */
    void setGains(float kp, float ki, float kd);

    /**
     * @brief 注册用户回调函数 (在计算Err后, 计算PID前调用)
     */
    void setUserFunc1(UserFunc_t func);

    /**
     * @brief 注册用户回调函数 (在计算P,I,D项后, 应用改进前调用)
     */
    void setUserFunc2(UserFunc_t func);

    // --- Getters ---
    [[nodiscard]] float getOutput() const { return _output; }
    [[nodiscard]] float getPout() const { return _p_out; }
    [[nodiscard]] float getIout() const { return _i_out; }
    [[nodiscard]] float getDout() const { return _d_out; }
    [[nodiscard]] float getError() const { return _err; }

private:
    // --- 私有辅助函数 (原C语言的 static f_... 函数) ---
    void trapezoidIntegral();
    void limitIntegral();
    // void derivativeOnMeasurement(); // <--- 已移除，逻辑合并到 calculate
    void changingIntegrationRate();
    void filterOutput();
    void filterDerivative();
    void limitOutput();
    void limitProportion();
    void handleError();

    // --- 私有成员 (原C语言的 PID_t 结构体字段) ---

    // 配置
    float _kp, _ki, _kd;
    float _max_out, _integral_limit, _deadband;
    float _coef_a, _coef_b;           // 变速积分参数
    float _output_lpf_rc;             // 输出LPF RC
    float _derivative_lpf_rc;         // 微分LPF RC
    uint16_t _ols_order;              // OLS 阶数
    uint8_t _improve;                 // 改进标志位
    ErrorHandler _error_handler{};      // 错误处理器
    UserFunc_t _user_func1 = nullptr; // 回调函数1
    UserFunc_t _user_func2 = nullptr; // 回调函数2

    // 状态
    float _ref = 0.0f;
    float _measure = 0.0f;
    float _err = 0.0f;
    float _last_err = 0.0f;
    float _p_out = 0.0f;
    float _i_out = 0.0f;
    float _d_out = 0.0f;
    float _i_term = 0.0f; // 当前帧的积分项
    float _last_i_term = 0.0f;
    float _output = 0.0f;
    float _last_output = 0.0f;
    float _last_d_out = 0.0f;
    float _last_measure = 0.0f;

    // 依赖
    uint32_t _dwt_cnt = 0; // 用于 DWT_Driver
    float _dt = 0.0f;      // DWT 计算出的时间间隔
    ols_t _ols; // OLS 实例
};

} // namespace pyro

#endif