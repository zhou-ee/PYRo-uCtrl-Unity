#ifndef __PYRO_CHASSIS_BASE_H__
#define __PYRO_CHASSIS_BASE_H__

#include "pyro_core_fsm.h"
#include "pyro_mutex.h"
#include "pyro_task.h"
#include <type_traits>

namespace pyro
{

struct cmd_base_t
{
    uint32_t timestamp;
    float vx, vy, wz;
    cmd_base_t() : timestamp(0), vx(0), vy(0), wz(0)
    {
    }
    virtual ~cmd_base_t() = default;
};

// =========================================================
// Template Chassis Base (Composition Version)
// =========================================================
template <typename CmdType> class chassis_base_t
{

    static_assert(std::is_base_of_v<cmd_base_t, CmdType>,
                  "CmdType MUST inherit from pyro::cmd_base_t");

  public:
    // 启动底盘任务
    void start();

    // 类型安全的命令下发
    void set_command(const CmdType &cmd);

    [[nodiscard]] mutex_t &get_mutex();

  protected:
    explicit chassis_base_t(
        const char *name = "chassis", uint16_t init_stack = 512,
        uint16_t loop_stack              = 256,
        task_base_t::priority_t priority = task_base_t::priority_t::HIGH);

    virtual ~chassis_base_t() = default;
    // -----------------------------------------------------
    // 业务逻辑接口 (由派生类实现)
    // -----------------------------------------------------
    virtual void init()               = 0;
    virtual void update_command()     = 0;
    virtual void update_feedback()    = 0;
    virtual void kinematics_solve()   = 0;
    virtual void chassis_control()    = 0;
    virtual void power_control()      = 0;
    virtual void send_motor_command() = 0;

    CmdType _cmd;
    mutex_t _mutex;

    template <typename T> class base_fsm_t : public fsm_t<T>
    {
        class active_fsm_t;
        class passive_fsm_t;
        static_assert(std::is_base_of_v<chassis_base_t, T>,
                      "T MUST inherit from pyro::chassis_base_t");
    };
    base_fsm_t<chassis_base_t> *_fsm;

  private:
    // -----------------------------------------------------
    // 内部任务代理类声明
    // 注意：因为 _task 是成员变量(非指针)，类定义必须在此处可见
    // 但方法的具体实现移到了 .tpp 中
    // -----------------------------------------------------
    class chassis_task_t final : public task_base_t
    {
      public:
        chassis_task_t(chassis_base_t *owner_ptr, const char *name,
                       uint16_t init_stack, uint16_t loop_stack,
                       priority_t priority);

      protected:
        void init() override;
        void run_loop() override;

      private:
        chassis_base_t *_owner;
    };

    // -----------------------------------------------------
    // 构造函数 (Private as per your code)
    // -----------------------------------------------------


    // 核心循环逻辑实现
    void run_loop_impl();

    // 组合成员变量
    chassis_task_t _task;

};

} // namespace pyro

// =========================================================
// 包含实现文件 (.tpp)
// =========================================================
#include "pyro_chassis_base.tpp"

#endif // __PYRO_CHASSIS_BASE_H__