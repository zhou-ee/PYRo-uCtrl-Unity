#ifndef __PYRO_DWT_DRV_H__
#define __PYRO_DWT_DRV_H__

#include "stdint.h"

namespace pyro
{

/**
 * @brief C++ DWT (Data Watchpoint and Trace) 高分辨率定时器驱动
 *
 * 这是一个静态类，为 ARM Cortex-M DWT CYCCNT 寄存器
 * 提供了单例接口，用于高精度计时。
 */
class dwt_drv_t
{
  public:
    // --- 构造/析构 ---
    // 删除构造函数，使该类成为纯静态类，无法被实例化。
    dwt_drv_t()                              = delete;
    dwt_drv_t(const dwt_drv_t &)            = delete;
    dwt_drv_t &operator=(const dwt_drv_t &) = delete;
    /**
     * @brief 替代C语言的 DWT_Time_t 结构体
     */
    struct Time
    {
        uint32_t s;  // 秒
        uint16_t ms; // 毫秒
        uint16_t us; // 微秒
    };

    /**
     * @brief 初始化DWT外设。必须在程序开始时调用一次。
     * @param cpu_freq_mhz 系统的CPU频率，单位 MHz。
     */
    static void init(uint32_t cpu_freq_mhz);

    /**
     * @brief 获取自 'cnt_last' 以来经过的时间增量（float, 单位秒）。
     * @param cnt_last 指向保存上一次计数值的变量的指针。
     * 该变量将被更新为当前的计数值。
     * @return 时间增量（float, 单位秒）。
     */
    static float getDeltaT(uint32_t *cnt_last);

    /**
     * @brief 获取自 'cnt_last' 以来经过的时间增量（double, 单位秒）。
     * @param cnt_last 指向保存上一次计数值的变量的指针。
     * 该变量将被更新为当前的计数值。
     * @return 时间增量（double, 单位秒）。
     */
    static double getDeltaT64(uint32_t *cnt_last);

    /**
     * @brief 获取自init()以来经过的总时间（float, 单位秒）。
     */
    static float getTimeline_s();

    /**
     * @brief 获取自init()以来经过的总时间（float, 单位毫秒）。
     */
    static float getTimeline_ms();

    /**
     * @brief 获取自init()以来经过的总时间（uint64_t, 单位微秒）。
     */
    static uint64_t getTimeline_us();

    /**
     * @brief 获取自init()以来经过的总时间（Time 结构体）。
     */
    static Time getTimeline();

    /**
     * @brief 阻塞延迟（float, 单位秒）。
     */
    static void delay_s(float seconds);

    /**
     * @brief 阻塞延迟（uint32_t, 单位微秒）。
     */
    static void delay_us(uint32_t microseconds);

    /**
     * @brief 获取当前原始的32位 DWT->CYCCNT 计数值。
     */
    static uint32_t getCurrentTicks();

  private:
    /**
     * @brief 更新64位周期计数器并处理32位计数器溢出。
     */
    static void updateCycleCount();

    /**
     * @brief 更新内部的 _sys_time 结构体。
     */
    static void updateSysTime();

    // --- 私有静态成员 (替换C语言的全局变量) ---
    inline static uint32_t _cpu_freq_hz{};
    inline static uint32_t _cpu_freq_hz_ms{};
    inline static uint32_t _cpu_freq_hz_us{};
    inline static uint32_t _cyccnt_round_count{}; // 32位计数器溢出次数
    inline static uint32_t _cyccnt_last{}; // 上一次的计数值，用于检测溢出
    inline static uint64_t _cyccnt64{};    // 64位总计数值
    inline static Time _sys_time{};        // 格式化的系统时间
};

} // namespace pyro

#endif