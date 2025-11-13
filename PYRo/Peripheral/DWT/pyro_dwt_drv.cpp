#include "pyro_dwt_drv.h"
#include "main.h"

namespace pyro
{
/**
 * @brief 替代C语言的 DWT_Init
 */
void dwt_drv_t::init(const uint32_t cpu_freq_mhz)
{
    // 使能DWT外设
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // DWT CYCCNT寄存器计数清0
    DWT->CYCCNT = 0u;

    // 使能Cortex-M DWT CYCCNT寄存器
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // 存储不同单位的频率
    _cpu_freq_hz        = cpu_freq_mhz * 1000000;
    _cpu_freq_hz_ms     = _cpu_freq_hz / 1000;
    _cpu_freq_hz_us     = _cpu_freq_hz / 1000000;

    // 重置所有计数器状态
    _cyccnt_round_count = 0;
    _cyccnt_last        = 0;
    _cyccnt64           = 0;
}

/**
 * @brief 替代C语言的 DWT_GetDeltaT
 */
float dwt_drv_t::getDeltaT(uint32_t *cnt_last)
{
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    // 计算增量，(uint32_t)转型处理了32位回绕
    const float dt =
        static_cast<float>(static_cast<uint32_t>(cnt_now - *cnt_last)) /
        static_cast<float>(_cpu_freq_hz);
    *cnt_last = cnt_now;

    // 注意：C版本中的 DWT_CNT_Update() 调用已被移除。
    // GetDeltaT 函数不应该修改全局的时间线状态。

    return dt;
}

/**
 * @brief 替代C语言的 DWT_GetDeltaT64
 */
double dwt_drv_t::getDeltaT64(uint32_t *cnt_last)
{
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    const double dt                 = static_cast<double>(cnt_now - *cnt_last) /
                      static_cast<double>(_cpu_freq_hz);
    *cnt_last = cnt_now;

    // 注意：C版本中的 DWT_CNT_Update() 调用已被移除。

    return dt;
}

/**
 * @brief 替代C语言的 DWT_SysTimeUpdate
 */
void dwt_drv_t::updateSysTime()
{
    // 必须先更新溢出计数器
    updateCycleCount();

    const volatile uint32_t cnt_now = DWT->CYCCNT;

    // 计算64位总计数值
    _cyccnt64 = static_cast<uint64_t>(_cyccnt_round_count) * 0x100000000ULL +
                static_cast<uint64_t>(cnt_now);

    // 分解为 s, ms, us
    const uint64_t CNT_TEMP1 = _cyccnt64 / _cpu_freq_hz;
    const uint64_t CNT_TEMP2 = _cyccnt64 - CNT_TEMP1 * _cpu_freq_hz;
    _sys_time.s              = static_cast<uint32_t>(CNT_TEMP1);
    _sys_time.ms = static_cast<uint16_t>(CNT_TEMP2 / _cpu_freq_hz_ms);
    const uint64_t CNT_TEMP3 =
        CNT_TEMP2 - static_cast<uint64_t>(_sys_time.ms) * _cpu_freq_hz_ms;
    _sys_time.us = static_cast<uint16_t>(CNT_TEMP3 / _cpu_freq_hz_us);
}

/**
 * @brief 替代C语言的 DWT_GetTimeline_s
 */
float dwt_drv_t::getTimeline_s()
{
    updateSysTime();
    return static_cast<float>(_sys_time.s) +
           static_cast<float>(_sys_time.ms) * 0.001f +
           static_cast<float>(_sys_time.us) * 0.000001f;
}

/**
 * @brief 替代C语言的 DWT_GetTimeline_ms
 */
float dwt_drv_t::getTimeline_ms()
{
    updateSysTime();
    return static_cast<float>(_sys_time.s) * 1000.0f +
           static_cast<float>(_sys_time.ms) +
           static_cast<float>(_sys_time.us) * 0.001f;
}

/**
 * @brief 替代C语言的 DWT_GetTimeline_us
 */
uint64_t dwt_drv_t::getTimeline_us()
{
    updateSysTime();
    return static_cast<uint64_t>(_sys_time.s) * 1000000 +
           static_cast<uint64_t>(_sys_time.ms) * 1000 +
           static_cast<uint64_t>(_sys_time.us);
}

/**
 * @brief [新增] 获取格式化的时间
 */
dwt_drv_t::Time dwt_drv_t::getTimeline()
{
    updateSysTime();
    return _sys_time;
}

/**
 * @brief 替代C语言的 DWT_CNT_Update (私有)
 */
void dwt_drv_t::updateCycleCount()
{
    const volatile uint32_t cnt_now = DWT->CYCCNT;
    if (cnt_now < _cyccnt_last)
    {
        _cyccnt_round_count++;
    }
    _cyccnt_last = cnt_now;
}

/**
 * @brief 替代C语言的 DWT_Delay
 */
void dwt_drv_t::delay_s(const float seconds)
{
    const uint32_t start_tick = DWT->CYCCNT;
    const auto delay_ticks =
        static_cast<uint32_t>(seconds * static_cast<float>(_cpu_freq_hz));

    while ((DWT->CYCCNT - start_tick) < delay_ticks)
    {
        // 忙等待 (Busy wait)
    }
}

/**
 * @brief 替代C语言的 DWT_Delay_us
 */
void dwt_drv_t::delay_us(const uint32_t microseconds)
{
    const uint32_t start_tick = DWT->CYCCNT;
    const auto delay_ticks    = static_cast<uint32_t>(
        static_cast<float>(microseconds) * static_cast<float>(_cpu_freq_hz_us));

    while ((DWT->CYCCNT - start_tick) < delay_ticks)
    {
        // 忙等待 (Busy wait)
    }
}

/**
 * @brief [新增] 辅助函数，获取当前原始计数值
 */
uint32_t dwt_drv_t::getCurrentTicks()
{
    return DWT->CYCCNT;
}

} // namespace pyro