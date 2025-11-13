#ifndef __PYRO_ALGO_OLS_H__
#define __PYRO_ALGO_OLS_H__

#include <cstdint>
#include <vector>

namespace pyro
{

/**
 * @brief 最小二乘法 (OLS) 线性回归 C++ 实现
 *
 * 这个类将 user_lib.c 中的C语言实现重构为一个面向对象的C++类。
 * 它使用 std::vector 自动管理内存 (RAII)，
 * 并提供了一个清晰的API来更新数据和获取回归结果。
 */
class ols_t
{
public:
    /**
     * @brief 构造 OLS 滤波器。
     * @param order 用于回归的样本数（窗口大小）。
     * 注意：order 必须大于1。
     */
    explicit ols_t(uint16_t order);

    /**
     * @brief 用一个新的数据点更新 OLS 滤波器。
     *
     * 此函数将新的 (deltax, y) 数据点添加到数据窗口中，
     * 推出最旧的数据点。然后它会重新计算
     * 斜率 (k)、截距 (b) 和平均绝对偏差。
     *
     * @param deltax 自上一个点以来经过的时间（或x的变化量）。
     * @param y 新的 y 值（信号值）。
     */
    void update(float deltax, float y);

    /**
     * @brief 获取计算出的导数（斜率 'k'）。
     * @return 线性回归的当前导数 (k)。
     */
    float getDerivative() const;

    /**
     * @brief 获取平滑后的信号值。
     * @return 最近的平滑 y 值，计算公式为 (k * x_last + b)。
     */
    float getSmooth() const;

    /**
     * @brief 获取平均绝对偏差。
     * @note 原C代码将其命名为 'StandardDeviation'，
     * 但计算方法是平均绝对偏差 (Mean Absolute Deviation)。
     * @return 当前回归线的平均绝对偏差。
     */
    float getMeanAbsoluteDeviation() const;

private:
    // 成员变量（使用 _ 前缀）
    uint16_t _order;
    uint32_t _count;

    std::vector<float> _x;
    std::vector<float> _y;

    float _k; // 斜率
    float _b; // 截距

    float _deviation; // 平均绝对偏差
};

} // namespace pyro

#endif // OLS_H