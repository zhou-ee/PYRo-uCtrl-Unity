#ifndef __PYRO_TASK_H__
#define __PYRO_TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include <cstdint>

namespace pyro
{

class task_base_t
{
public:
    // 将枚举移至 public，对应数值 0-6
    enum class priority_t : uint8_t
    {
        IDLE,         // 0: 空闲级 (映射到 0)
        LOW,          // 1: 低优先级
        BELOW_NORMAL, // 2: 次正常级 (新增)
        NORMAL,       // 3: 正常级
        ABOVE_NORMAL, // 4: 高正常级 (新增)
        HIGH,         // 5: 高优先级
        REALTIME,     // 6: 实时级 (映射到 configMAX_PRIORITIES - 1)
    };

    // 构造函数参数改为 priority_t
    task_base_t(const char *name, uint16_t init_stack, uint16_t loop_stack,
                priority_t priority);
    virtual ~task_base_t();

    void start();
    void stop();

protected:
    virtual void init()     = 0;
    virtual void run_loop() = 0;

    TaskHandle_t _loop_task_handle;

private:
    const char *_task_name;
    uint16_t _init_stack_depth;
    uint16_t _loop_stack_depth;
    priority_t _priority; // 类型同步修改

    static void init_entry_point(void *arg);
    static void loop_entry_point(void *arg);

    // 内部转换工具
    static UBaseType_t convert_priority(priority_t p);
};

} // namespace pyro
#endif