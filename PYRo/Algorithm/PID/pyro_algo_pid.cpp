#include "pyro_algo_pid.h"
#include "pyro_dwt_drv.h"
#include <cmath>


namespace pyro
{

// --- 构造函数实现 ---

// 构造函数 1: 基本PID (委托给构造函数3)
pid_t::pid_t(const float kp, const float ki, const float kd,
             const float integral_limit, const float max_out,
             const uint8_t improve)
    : pid_t(max_out,        // max_out
            integral_limit, // integral_limit
            0.0f,           // deadband
            kp, ki, kd,     // Kp, Ki, Kd
            0.0f, 0.0f,     // A, B (变速积分)
            0.0f, 0.0f,     // output_lpf_rc, derivative_lpf_rc
            0,              // ols_order
            improve)        // improve
{
}

// 构造函数 2: 带滤波和OLS的PID (委托给构造函数3)
pid_t::pid_t(const float kp, const float ki, const float kd,
             const float integral_limit, const float max_out,
             const float output_lpf_rc, const float derivative_lpf_rc,
             const uint16_t ols_order,
             const uint8_t improve)
    : pid_t(max_out,           // max_out
            integral_limit,    // integral_limit
            0.0f,              // deadband
            kp, ki, kd,        // Kp, Ki, Kd
            0.0f, 0.0f,        // A, B (变速积分)
            output_lpf_rc,     // output_lpf_rc
            derivative_lpf_rc, // derivative_lpf_rc
            ols_order,         // ols_order
            improve)           // improve
{
}

// 构造函数 3: 完整版 (主构造函数)
pid_t::pid_t(const float max_out, const float integral_limit,
             const float deadband, const float kp, const float ki,
             const float kd, const float A, const float B,
             const float output_lpf_rc, const float derivative_lpf_rc,
             const uint16_t ols_order,
             const uint8_t improve)
    : // --- C++ 成员初始化列表 ---
      _kp(kp), _ki(ki), _kd(kd), _max_out(max_out),
      _integral_limit(integral_limit), _deadband(deadband), _coef_a(A),
      _coef_b(B), _output_lpf_rc(output_lpf_rc),
      _derivative_lpf_rc(derivative_lpf_rc), _ols_order(ols_order),
      _improve(improve), _ols(ols_order) // <-- 关键: 在此处构造 OLS 实例
{
    // 构造函数体
    clear(); // 调用 clear 来初始化所有状态变量
    _error_handler = {0, ErrorType::NONE};
}

// --- 公共方法 ---

float pid_t::calculate(const float measure, const float ref)
{
    if (_improve & Improvement::ErrorHandle)
    {
        handleError();
    }

    _dt = dwt_drv_t::getDeltaT(&_dwt_cnt);

    // 防止 _dt 过小或为0导致除零
    if (_dt < 1e-9f)
    {
        _last_measure = measure;
        _last_err     = _err;
        return _output;
    }

    _measure = measure;
    _ref     = ref;
    _err     = _ref - _measure; // 标准PID误差

    if (_user_func1)
    {
        _user_func1(this);
    }

    if (std::fabs(_err) > _deadband)
    {
        // --- 计算 P, I ---
        _p_out  = _kp * _err;
        _i_term = _ki * _err * _dt;

        // --- D Term Calculation (FIXED LOGIC) ---
        // 检查是D-on-M 还是 D-on-E
        if (_improve & Improvement::Derivative_On_Measurement)
        {
            // D-on-M (微分先行)
            if (_ols_order > 2)
            {
                _ols.update(_dt, -_measure); // 1. 仅用 -Measure 更新
                _d_out = _kd * _ols.getDerivative();
            }
            else
            {
                _d_out = _kd * (_last_measure - _measure) / _dt;
            }
        }
        else
        {
            // D-on-Error (标准微分)
            if (_ols_order > 2)
            {
                _ols.update(_dt, _err); // 1. 仅用 Error 更新
                _d_out = _kd * _ols.getDerivative();
            }
            else
            {
                _d_out = _kd * (_err - _last_err) / _dt;
            }
        }
        // --- End of D Term Calculation ---

        if (_user_func2)
        {
            _user_func2(this);
        }

        // --- 应用PID改进 ---
        if (_improve & Improvement::Trapezoid_Intergral)
        {
            trapezoidIntegral();
        }
        if (_improve & Improvement::ChangingIntegrationRate)
        {
            changingIntegrationRate();
        }

        // D-on-M 逻辑已在D项计算中处理，此处无需再调用

        if (_improve & Improvement::DerivativeFilter)
        {
            filterDerivative();
        }

        if (_improve & Improvement::Integral_Limit)
        {
            limitIntegral();
        }

        // 累加积分
        _i_out += _i_term;

        // --- 计算总输出 ---
        _output = _p_out + _i_out + _d_out;

        if (_improve & Improvement::OutputFilter)
        {
            filterOutput();
        }

        limitOutput();

        limitProportion();
    }

    // --- 更新 'Last' 状态 ---
    _last_measure = _measure;
    _last_output  = _output;
    _last_d_out   = _d_out;
    _last_err     = _err;
    _last_i_term  = _i_term;

    return _output;
}

void pid_t::clear()
{
    _ref          = 0.0f;
    _measure      = 0.0f;
    _err          = 0.0f;
    _last_err     = 0.0f;
    _p_out        = 0.0f;
    _i_out        = 0.0f;
    _d_out        = 0.0f;
    _i_term       = 0.0f;
    _last_i_term  = 0.0f;
    _output       = 0.0f;
    _last_output  = 0.0f;
    _last_d_out   = 0.0f;
    _last_measure = 0.0f;
    _dwt_cnt      = 0; // 重置DWT计数器
    _dt           = 0.0f;
}

void pid_t::setGains(const float kp, const float ki, const float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void pid_t::setUserFunc1(const UserFunc_t func)
{
    _user_func1 = func;
}

void pid_t::setUserFunc2(const UserFunc_t func)
{
    _user_func2 = func;
}

// --- 私有辅助函数 (PID 改进) ---

void pid_t::trapezoidIntegral()
{
    _i_term = _ki * ((_err + _last_err) / 2.0f) * _dt;
}

void pid_t::changingIntegrationRate()
{
    if (_err * _i_out > 0) // 积分呈累积趋势
    {
        if (std::fabs(_err) <= _coef_b)
        {
            return; // 全速积分
        }
        if (std::fabs(_err) <= (_coef_a + _coef_b))
        {
            _i_term *= (_coef_a - std::fabs(_err) + _coef_b) / _coef_a;
        }
        else
        {
            _i_term = 0.0f;
        }
    }
}

void pid_t::limitIntegral()
{
    const float temp_Iout   = _i_out + _i_term;
    const float temp_Output = _p_out + temp_Iout + _d_out; // 预计算的输出

    // 抗饱和 (Anti-Windup)
    if (std::fabs(temp_Output) > _max_out)
    {
        if (_err * _i_out > 0) // 积分呈累积趋势
        {
            _i_term = 0.0f; // 停止积分
        }
    }

    // 积分项限幅
    if (temp_Iout > _integral_limit)
    {
        _i_term = 0.0f;
        _i_out  = _integral_limit;
    }
    else if (temp_Iout < -_integral_limit)
    {
        _i_term = 0.0f;
        _i_out  = -_integral_limit;
    }
}

// derivativeOnMeasurement() 函数已被移除

void pid_t::filterDerivative()
{
    if (_derivative_lpf_rc > 0.0f)
    {
        _d_out = _d_out * _dt / (_derivative_lpf_rc + _dt) +
                 _last_d_out * _derivative_lpf_rc / (_derivative_lpf_rc + _dt);
    }
}

void pid_t::filterOutput()
{
    if (_output_lpf_rc > 0.0f)
    {
        _output = _output * _dt / (_output_lpf_rc + _dt) +
                  _last_output * _output_lpf_rc / (_output_lpf_rc + _dt);
    }
}

void pid_t::limitOutput()
{
    if (_output > _max_out)
    {
        _output = _max_out;
    }
    else if (_output < -_max_out)
    {
        _output = -_max_out;
    }
}

void pid_t::limitProportion()
{
    if (_p_out > _max_out)
    {
        _p_out = _max_out;
    }
    else if (_p_out < -_max_out)
    {
        _p_out = -_max_out;
    }
}

void pid_t::handleError()
{
    if (_output < _max_out * 0.001f || std::fabs(_ref) < 0.0001f)
    {
        return;
    }

    // 检查除零风险
    const float ref_abs = std::fabs(_ref);
    if (ref_abs > 1e-6f) // 避免除以一个非常小或为零的 ref
    {
        if ((std::fabs(_ref - _measure) / ref_abs) > 0.95f)
        {
            _error_handler.ERRORCount++;
        }
        else
        {
            _error_handler.ERRORCount = 0;
        }
    }
    else
    {
        _error_handler.ERRORCount = 0; // ref 接近零，不计为堵转
    }


    if (_error_handler.ERRORCount > 500)
    {
        _error_handler.ERRORType = ErrorType::Motor_Blocked;
    }
}

} // namespace pyro