#pragma once
namespace pyro
{

// =========================================================
// chassis_base_t 成员函数实现
// =========================================================

template <typename CmdType>
chassis_base_t<CmdType>::chassis_base_t(const char *name, uint16_t init_stack,
                                        uint16_t loop_stack,
                                        task_base_t::priority_t priority)
    : _task(this, name, init_stack, loop_stack, priority)
{
}

template <typename CmdType> void chassis_base_t<CmdType>::start()
{
    _task.start();
}

template <typename CmdType>
void chassis_base_t<CmdType>::set_command(const CmdType &cmd)
{
    scoped_mutex_t lock(_mutex);
    _cmd = cmd;
}

template <typename CmdType> mutex_t &chassis_base_t<CmdType>::get_mutex()
{
    return _mutex;
}


template <typename CmdType> void chassis_base_t<CmdType>::run_loop_impl()
{
    TickType_t xLastWakeTime        = xTaskGetTickCount();
    constexpr TickType_t xFrequency = pdMS_TO_TICKS(1);

    while (true)
    {
        {
            scoped_mutex_t lock(_mutex);
            update_command();
        }
        _fsm->execute(this);
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =========================================================
// 内部类 chassis_task_proxy 成员函数实现
// =========================================================

// 构造函数
template <typename CmdType>
chassis_base_t<CmdType>::chassis_task_t::chassis_task_t(
    chassis_base_t *owner_ptr, const char *name, const uint16_t init_stack,
    const uint16_t loop_stack, const priority_t priority)
    : task_base_t(name, init_stack, loop_stack, priority), _owner(owner_ptr)
{
}

// init 代理
template <typename CmdType> void chassis_base_t<CmdType>::chassis_task_t::init()
{
    if (_owner)
    {
        _owner->init();
    }
}

// run_loop 代理
template <typename CmdType>
void chassis_base_t<CmdType>::chassis_task_t::run_loop()
{
    if (_owner)
    {
        _owner->run_loop_impl();
    }
}

} // namespace pyro