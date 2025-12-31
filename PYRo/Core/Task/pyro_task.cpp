#include "pyro_task.h"

namespace pyro
{

// 构造函数：初始化自定义枚举
task_base_t::task_base_t(const char *name, const uint16_t init_stack,
                         const uint16_t loop_stack, const priority_t priority)
    : _loop_task_handle(nullptr), _task_name(name),
      _init_stack_depth(init_stack), _loop_stack_depth(loop_stack),
      _priority(priority)
{
}

task_base_t::~task_base_t()
{
    stop();
}

void task_base_t::start()
{
    if (_loop_task_handle != nullptr)
    {
        return;
    }

    // 创建任务时进行优先级转换
    xTaskCreate(init_entry_point, "init_tmp", _init_stack_depth, this,
                convert_priority(_priority), nullptr);
}

void task_base_t::stop()
{
    if (_loop_task_handle != nullptr)
    {
        vTaskDelete(_loop_task_handle);
        _loop_task_handle = nullptr;
    }
}

void task_base_t::init_entry_point(void *arg)
{
    auto *self = static_cast<task_base_t *>(arg);

    if (self)
    {
        self->init();

        if (self->_loop_stack_depth > 0)
        {
            // 创建 Loop 任务时同步转换优先级
            xTaskCreate(loop_entry_point, self->_task_name,
                        self->_loop_stack_depth, self,
                        convert_priority(self->_priority),
                        &self->_loop_task_handle);
        }
    }
    vTaskDelete(nullptr);
}

void task_base_t::loop_entry_point(void *arg)
{
    auto *self = static_cast<task_base_t *>(arg);
    if (self)
    {
        self->run_loop();
        // 正常退出时的资源回收逻辑

        self->_loop_task_handle = nullptr;
        vTaskDelete(nullptr);
    }
    vTaskDelete(nullptr);
}

/**
 * @brief 最简高效的优先级映射实现
 * 公式：(priority * (MAX_PRIORITIES - 1)) / 6
 * 确保 0 对应最低优先级，6 对应系统允许的最大值
 */
UBaseType_t task_base_t::convert_priority(priority_t p)
{
    // 强制转换为整数进行运算，先乘后除防止精度损失
    // FreeRTOS 允许的最大优先级数值为 MAX_PRIORITIES - 1
    return static_cast<UBaseType_t>(p) * (configMAX_PRIORITIES - 1) / 6;
}

} // namespace pyro