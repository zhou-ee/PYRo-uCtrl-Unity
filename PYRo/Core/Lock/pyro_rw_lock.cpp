#include "pyro_rw_lock.h"

namespace pyro
{

RWLock::RWLock() :
    reader_count_(0),
    writer_waiting_count_(0)
{
    internal_mutex_ = xSemaphoreCreateMutex();
    read_gate_ = xSemaphoreCreateBinary();
    write_gate_ = xSemaphoreCreateBinary();

    configASSERT(internal_mutex_ != nullptr);
    configASSERT(read_gate_ != nullptr);
    configASSERT(write_gate_ != nullptr);

    // 初始化信号量为“可用”状态
    xSemaphoreGive(read_gate_);
    xSemaphoreGive(write_gate_);
}

RWLock::~RWLock()
{
    vSemaphoreDelete(internal_mutex_);
    vSemaphoreDelete(read_gate_);
    vSemaphoreDelete(write_gate_);
}

// ----------------------------------------------------------------
// 阻塞式 (无限等待) API - 已修正
// ----------------------------------------------------------------

void RWLock::readLock()
{
    // 1. 等待 "读者大门" (写优先)
    xSemaphoreTake(read_gate_, portMAX_DELAY);

    // 2. 保护计数器 (极短的临界区)
    xSemaphoreTake(internal_mutex_, portMAX_DELAY);
    reader_count_++;
    bool first_reader = (reader_count_ == 1);
    xSemaphoreGive(internal_mutex_); // 立即释放

    // 3. 如果是第一个读者，获取 "写者大门" 以阻止写者
    //    (这个等待现在发生在 internal_mutex_ 之外)
    if (first_reader) {
        xSemaphoreTake(write_gate_, portMAX_DELAY);
    }

    // 4. 归还 "读者大门"，允许其他读者进入
    xSemaphoreGive(read_gate_);
}

void RWLock::readUnlock()
{
    // (此函数逻辑正确，临界区已很短)
    xSemaphoreTake(internal_mutex_, portMAX_DELAY);
    reader_count_--;
    if (reader_count_ == 0) {
        // 最后一个读者释放 "写者大门"
        xSemaphoreGive(write_gate_);
    }
    xSemaphoreGive(internal_mutex_);
}

void RWLock::writeLock()
{
    bool first_writer = false;

    // 1. 保护计数器 (极短的临界区)
    xSemaphoreTake(internal_mutex_, portMAX_DELAY);
    writer_waiting_count_++;
    if (writer_waiting_count_ == 1) {
        first_writer = true;
    }
    xSemaphoreGive(internal_mutex_); // 立即释放

    // 2. 如果是第一个等待的写者，拿走 "读者大门"
    //    (这个等待现在发生在 internal_mutex_ 之外)
    if (first_writer) {
        xSemaphoreTake(read_gate_, portMAX_DELAY);
    }

    // 3. 等待获取 "写者大门" (独占访问)
    xSemaphoreTake(write_gate_, portMAX_DELAY);
}

void RWLock::writeUnlock()
{
    // (此函数逻辑正确，临界区已很短)
    xSemaphoreGive(write_gate_); // 释放 "写者大门"

    xSemaphoreTake(internal_mutex_, portMAX_DELAY);
    writer_waiting_count_--;
    if (writer_waiting_count_ == 0) {
        // 最后一个写者归还 "读者大门"
        xSemaphoreGive(read_gate_);
    }
    xSemaphoreGive(internal_mutex_);
}


// ----------------------------------------------------------------
// 带超时的 API - 已修正
// ----------------------------------------------------------------

bool RWLock::readLock(TickType_t timeout_ticks)
{
    TickType_t start_time = xTaskGetTickCount();
    TickType_t remaining_time = timeout_ticks;
    TickType_t elapsed_time = 0;
    bool first_reader = false;

    // 1. 尝试获取 "读者大门"
    if (xSemaphoreTake(read_gate_, remaining_time) == pdFALSE) {
        return false; // 超时
    }

    // 2. 保护计数器 (极短的临界区)
    xSemaphoreTake(internal_mutex_, portMAX_DELAY);
    reader_count_++;
    if (reader_count_ == 1) {
        first_reader = true;
    }
    xSemaphoreGive(internal_mutex_); // 立即释放

    // 3. 如果是第一个读者，尝试获取 "写者大门"
    if (first_reader) {
        // 计算剩余时间
        elapsed_time = xTaskGetTickCount() - start_time;
        if (elapsed_time >= timeout_ticks) {
            remaining_time = 0; // 时间已用完
        } else {
            remaining_time = timeout_ticks - elapsed_time;
        }

        if (xSemaphoreTake(write_gate_, remaining_time) == pdFALSE) {
            // 超时，必须 "撤销"
            xSemaphoreTake(internal_mutex_, portMAX_DELAY);
            reader_count_--; // 撤销计数
            xSemaphoreGive(internal_mutex_);

            xSemaphoreGive(read_gate_); // 归还大门
            return false;
        }
    }

    // 4. 归还 "读者大门"
    xSemaphoreGive(read_gate_);
    return true; // 成功
}


bool RWLock::writeLock(TickType_t timeout_ticks)
{
    const TickType_t start_time = xTaskGetTickCount();
    TickType_t remaining_time = timeout_ticks;
    TickType_t elapsed_time = 0;
    bool first_writer = false;

    // 1. 保护计数器 (极短的临界区)
    xSemaphoreTake(internal_mutex_, remaining_time);
    writer_waiting_count_++;
    if (writer_waiting_count_ == 1) {
        first_writer = true;
    }
    xSemaphoreGive(internal_mutex_); // 立即释放

    // 2. 如果是第一个写者，尝试拿走 "读者大门"
    if (first_writer) {
        // 计算剩余时间
        elapsed_time = xTaskGetTickCount() - start_time;
        if (elapsed_time >= timeout_ticks) {
            remaining_time = 0; // 时间已用完
        } else {
            remaining_time = timeout_ticks - elapsed_time;
        }

        if (xSemaphoreTake(read_gate_, remaining_time) == pdFALSE) {
            // 超时，"撤销"
            xSemaphoreTake(internal_mutex_, portMAX_DELAY);
            writer_waiting_count_--; // 撤销计数
            xSemaphoreGive(internal_mutex_);
            return false;
        }
    }

    // 3. 尝试获取 "写者大门" (独占访问)
    elapsed_time = xTaskGetTickCount() - start_time;
    if (elapsed_time >= timeout_ticks) {
        remaining_time = 0; // 时间已用完
    } else {
        remaining_time = timeout_ticks - elapsed_time;
    }

    if (xSemaphoreTake(write_gate_, remaining_time) == pdFALSE) {
        // 超时，必须执行 "撤销"
        xSemaphoreTake(internal_mutex_, portMAX_DELAY);
        writer_waiting_count_--; // 减少等待计数

        // 如果我们是第一个写者，并且现在没有其他写者在等待
        // 我们必须归还 "读者大门"
        if (first_writer && writer_waiting_count_ == 0) {
            xSemaphoreGive(read_gate_);
        }
        xSemaphoreGive(internal_mutex_);
        return false;
    }

    // 成功获取写锁
    return true;
}
}
