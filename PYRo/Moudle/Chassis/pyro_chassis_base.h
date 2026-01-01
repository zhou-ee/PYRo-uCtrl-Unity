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
    enum class mode_t : uint8_t { ZERO_FORCE, ACTIVE } mode;
    uint32_t timestamp;
    float vx, vy, wz;
    cmd_base_t() : mode(mode_t::ZERO_FORCE), timestamp(0), vx(0), vy(0), wz(0)
    {
    }
    virtual ~cmd_base_t() = default;
};

// =========================================================
// Template Chassis Base (CRTP Singleton Version)
// 增加 Derived 参数，让基类知道它是被谁继承的
// =========================================================
template <typename Derived, typename CmdType>
class chassis_base_t
{
    static_assert(std::is_base_of_v<cmd_base_t, CmdType>,
                  "CmdType MUST inherit from pyro::cmd_base_t");

  public:
    /**
     * @brief 获取唯一实例 (Meyers Singleton)
     * 外部只能通过这个函数获取对象引用
     */
    static Derived *instance()
    {
        static Derived _instance_obj;// NOLINT safety
        return &_instance_obj;
    }

    // =====================================================
    // 删除拷贝和移动操作 (严防死守，确保单例)
    // =====================================================
    chassis_base_t(const chassis_base_t &) = delete;
    chassis_base_t &operator=(const chassis_base_t &) = delete;
    chassis_base_t(chassis_base_t &&) = delete;
    chassis_base_t &operator=(chassis_base_t &&) = delete;

    // 启动底盘任务
    void start();

    // 类型安全的命令下发
    void set_command(const CmdType &cmd);

    [[nodiscard]] mutex_t &get_mutex();

  protected:
    // 构造函数设为 protected，只有派生类能调用
    explicit chassis_base_t(
        const char *name = "chassis", uint16_t init_stack = 512,
        uint16_t loop_stack              = 256,
        task_base_t::priority_t priority = task_base_t::priority_t::HIGH);

    virtual ~chassis_base_t()          = default;

    // -----------------------------------------------------
    // 业务逻辑接口 (由派生类实现)
    // -----------------------------------------------------
    virtual void _init()               = 0;
    virtual void _update_feedback()    = 0;
    virtual void _fsm_execute()        = 0;
    // virtual void _update_feedback()    = 0;
    // virtual void _kinematics_solve()   = 0;
    // virtual void _chassis_control()    = 0;
    // virtual void _power_control()      = 0;
    // virtual void _send_motor_command() = 0;

    CmdType _cmd[2];
    uint8_t _read_index{0};

  private:
    // -----------------------------------------------------
    // 内部任务代理类声明
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

    void _update_command();
    void _run_loop_impl();

    // 组合成员变量
    chassis_task_t _task;
    bool _cmd_updated{false};
    mutex_t _mutex;
};

} // namespace pyro

#include "pyro_chassis_base.tpp"

#endif // __PYRO_CHASSIS_BASE_H__