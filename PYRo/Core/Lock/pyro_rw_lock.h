#ifndef __PYRO_RW_LOCK_H__
#define __PYRO_RW_LOCK_H__

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

namespace pyro
{

/**
 * @brief 基于FreeRTOS的C++读写锁（写优先）
 *
 * 该类实现了一个“写优先”的读写锁，并支持基于Tick的超时。
 * - 允许多个读线程同时访问。
 * - 只允许一个写线程访问，且读写互斥。
 * - “写优先”：当一个写线程请求锁时，
 * 任何新的读线程将被阻塞，直到所有等待的写线程完成。
 */
class RWLock {
public:
    RWLock();
    ~RWLock();

    // 禁用拷贝构造和拷贝赋值
    RWLock(const RWLock&) = delete;
    RWLock& operator=(const RWLock&) = delete;

    // ----------------------------------------------------------------
    // 阻塞式 (无限等待) API
    // ----------------------------------------------------------------

    /**
     * @brief 请求读锁 (无限期等待)
     */
    void readLock();

    /**
     * @brief 释放读锁
     */
    void readUnlock();

    /**
     * @brief 请求写锁 (无限期等待)
     */
    void writeLock();

    /**
     * @brief 释放写锁
     */
    void writeUnlock();


    // ----------------------------------------------------------------
    // 带超时的 API
    // ----------------------------------------------------------------

    /**
     * @brief 尝试请求读锁，带超时
     * @param timeout_ticks 等待的 FreeRTOS Tick 数量
     * @return true 如果成功获取锁, false 如果超时
     */
    bool readLock(TickType_t timeout_ticks);

    /**
     * @brief 尝试请求写锁，带超时
     * @param timeout_ticks 等待的 FreeRTOS Tick 数量
     * @return true 如果成功获取锁, false 如果超时
     */
    bool writeLock(TickType_t timeout_ticks);


private:
    SemaphoreHandle_t internal_mutex_;       // 用于保护内部计数器的互斥锁
    SemaphoreHandle_t read_gate_;            // 读者“大门”信号量
    SemaphoreHandle_t write_gate_;           // 写者“大门”信号量
    volatile int reader_count_;              // 当前正在读取的读者数量
    volatile int writer_waiting_count_;      // 正在等待的写者数量
};


// ----------------------------------------------------------------------------
// RAII (Scoped Lock) 辅助类 (已更新)
// ----------------------------------------------------------------------------

/**
 * @brief RAII辅助类，用于自动获取和释放“读锁”
 */
class ReadScopeLock {
public:
    /**
     * @brief 构造函数 (无限等待)
     */
    explicit ReadScopeLock(RWLock& lock) : lock_(lock), is_locked_(true) {
        lock_.readLock();
    }

    /**
     * @brief 构造函数 (带超时, Ticks)
     */
    ReadScopeLock(RWLock& lock, const TickType_t timeout_ticks) : lock_(lock) {
        is_locked_ = lock_.readLock(timeout_ticks);
    }

    /**
     * @brief 析构函数，如果锁已获取，则释放
     */
    ~ReadScopeLock() {
        if (is_locked_) {
            lock_.readUnlock();
        }
    }

    /**
     * @brief 检查锁是否成功获取
     * @return true 如果锁被持有, false 如果超时
     */
    bool isLocked() const { return is_locked_; }

    // 禁用拷贝
    ReadScopeLock(const ReadScopeLock&) = delete;
    ReadScopeLock& operator=(const ReadScopeLock&) = delete;

private:
    RWLock& lock_;
    bool is_locked_;
};

/**
 * @brief RAII辅助类，用于自动获取和释放“写锁”
 */
class WriteScopeLock {
public:
    /**
     * @brief 构造函数 (无限等待)
     */
    explicit WriteScopeLock(RWLock& lock) : lock_(lock), is_locked_(true) {
        lock_.writeLock();
    }

    /**
     * @brief 构造函数 (带超时, Ticks)
     */
    WriteScopeLock(RWLock& lock, const TickType_t timeout_ticks) : lock_(lock) {
        is_locked_ = lock_.writeLock(timeout_ticks);
    }

    /**
     * @brief 析构函数，如果锁已获取，则释放
     */
    ~WriteScopeLock() {
        if (is_locked_) {
            lock_.writeUnlock();
        }
    }

    /**
     * @brief 检查锁是否成功获取
     * @return true 如果锁被持有, false 如果超时
     */
    bool isLocked() const { return is_locked_; }

    // 禁用拷贝
    WriteScopeLock(const WriteScopeLock&) = delete;
    WriteScopeLock& operator=(const WriteScopeLock&) = delete;

private:
    RWLock& lock_;
    bool is_locked_;
};

}
#endif
