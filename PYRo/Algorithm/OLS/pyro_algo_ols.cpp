#include "pyro_algo_ols.h"
#include <cmath>   // 用于 fabsf

namespace pyro
{

/**
 * @brief 构造函数，替代 OLS_Init
 */
ols_t::ols_t(uint16_t order)
    : _order(order),
      _count(0),
      _k(0.0f),
      _b(0.0f),
      _deviation(0.0f)
{
    // C++ 方式：使用 vector::resize，它会自动处理内存分配和初始化
    // 这取代了 user_malloc 和 memset
    // 我们至少需要2个点来做线性回归
    if (_order < 2)
    {
        _order = 2;
    }

    _x.resize(_order, 0.0f);
    _y.resize(_order, 0.0f);
}

/**
 * @brief 更新函数，整合了 OLS_Update, OLS_Derivative, OLS_Smooth 的核心逻辑
 */
void ols_t::update(float deltax, float y)
{
    // 1. 数据窗口平移（滑动窗口）
    // 这是对C代码中循环的直接翻译
    float temp = _x[1];
    for (uint16_t i = 0; i < _order - 1; ++i)
    {
        _x[i] = _x[i + 1] - temp;
        _y[i] = _y[i + 1];
    }
    // 添加新数据
    _x[_order - 1] = _x[_order - 2] + deltax;
    _y[_order - 1] = y;

    if (_count < _order)
    {
        _count++;
    }

    // 2. 重新计算统计和
    // 使用局部变量，而不是像C代码中那样的结构体成员 t[4]
    float t[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    // 循环的起始点与C代码一致
    uint16_t start_index = _order - _count;
    for (uint16_t i = start_index; i < _order; ++i)
    {
        t[0] += _x[i] * _x[i]; // sum(x^2)
        t[1] += _x[i];         // sum(x)
        t[2] += _x[i] * _y[i]; // sum(xy)
        t[3] += _y[i];         // sum(y)
    }

    // 3. 计算 k (斜率) 和 b (截距)
    // 复制C代码中的公式 (t[0]*Order - t[1]*t[1])
    float denominator = (t[0] * static_cast<float>(_order) - t[1] * t[1]);

    // **安全检查**: C代码中缺失了除零保护
    if (std::fabs(denominator) > 1e-9f) // 使用一个小的 epsilon 来避免浮点数比较问题
    {
        _k = (t[2] * static_cast<float>(_order) - t[1] * t[3]) / denominator;
        _b = (t[0] * t[3] - t[1] * t[2]) / denominator;
    }
    else
    {
        // 分母为零（所有 x 值都相同），无法计算斜率
        _k = 0.0f;
        // 如果k=0, b 应该是 y 的平均值
        _b = (_count > 0) ? (t[3] / static_cast<float>(_count)) : 0.0f;
    }


    // 4. 计算偏差（C代码中称为 StandardDeviation）
    _deviation = 0.0f;
    for (uint16_t i = start_index; i < _order; ++i)
    {
        _deviation += std::fabs(_k * _x[i] + _b - _y[i]);
    }

    // 复制C代码中的逻辑（除以 Order 而不是 Count）
    _deviation /= static_cast<float>(_order);
}

/**
 * @brief Getter: 获取导数 (k)
 */
float ols_t::getDerivative() const
{
    return _k;
}

/**
 * @brief Getter: 获取平滑值 (y = k*x + b)
 */
float ols_t::getSmooth() const
{
    // 使用 .back() 获取 std::vector 的最后一个元素
    // 这等同于C代码中的 OLS->x[OLS->Order - 1]
    return _k * _x.back() + _b;
}

/**
 * @brief Getter: 获取平均绝对偏差
 */
float ols_t::getMeanAbsoluteDeviation() const
{
    return _deviation;
}

} // namespace pyro