#pragma once
namespace pyro
{

// =========================================================
// chassis_base_t 成员函数实现
// =========================================================

template <typename Derived, typename CmdType>
chassis_base_t<Derived, CmdType>::chassis_base_t(const char *name, uint16_t init_stack,
                                        uint16_t loop_stack,
                                        task_base_t::priority_t priority)
    : _task(this, name, init_stack, loop_stack, priority)
{
}

template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::start()
{
    _task.start();
}

template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::set_command(const CmdType &cmd)
{
    scoped_mutex_t lock(_mutex);
    _cmd[1-_read_index] = cmd;
    _cmd_updated      = true;
}

template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::_update_command()
{
    if (_cmd_updated)
    {
        scoped_mutex_t lock(_mutex);
        _read_index    = 1 - _read_index;
        _cmd_updated   = false;
    }
}

template <typename Derived, typename CmdType>
mutex_t &chassis_base_t<Derived, CmdType>::get_mutex()
{
    return _mutex;
}

template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::_run_loop_impl()
{
    TickType_t xLastWakeTime        = xTaskGetTickCount();
    constexpr TickType_t xFrequency = pdMS_TO_TICKS(1);

    while (true)
    {
        _update_command();
        _update_feedback();
        _fsm_execute();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// =========================================================
// 内部类 chassis_task_t 成员函数实现
// =========================================================

// 构造函数
template <typename Derived, typename CmdType>
chassis_base_t<Derived, CmdType>::chassis_task_t::chassis_task_t(
    chassis_base_t *owner_ptr, const char *name, const uint16_t init_stack,
    const uint16_t loop_stack, const priority_t priority)
    : task_base_t(name, init_stack, loop_stack, priority), _owner(owner_ptr)
{
}

// init 代理
template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::chassis_task_t::init()
{
    if (_owner)
    {
        _owner->_init();
    }
}

// run_loop 代理
template <typename Derived, typename CmdType>
void chassis_base_t<Derived, CmdType>::chassis_task_t::run_loop()
{
    if (_owner)
    {
        _owner->_run_loop_impl();
    }
}

} // namespace pyro