/**
 * @file pyro_uart_drv.h
 * @brief Header file for the PYRO C++ UART Driver class.
 *
 * This file defines the `pyro::uart_drv_t` class, which encapsulates the
 * STM32 HAL UART functionality, including DMA double-buffering for reception
 * and integration with FreeRTOS for asynchronous event handling.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-10-09
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_UART_DRV_H__
#define __PYRO_UART_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"

#include "pyro_core_def.h"

#include "FreeRTOS.h"
#include "message_buffer.h"

#include "functional"
#include "map"
#include "vector"

namespace pyro
{
class uart_drv_t;
extern pyro::uart_drv_t uart1;
extern pyro::uart_drv_t uart5;
/* Class Definition ----------------------------------------------------------*/
/**
 * @brief C++ class to encapsulate the STM32 HAL UART driver functionality.
 *
 * It manages double-buffering for DMA reception, integrates with FreeRTOS
 * for yielding from ISRs, and uses a static map to dispatch HAL callbacks
 * to the correct C++ instance.
 */
class uart_drv_t
{
    /* Private Types ---------------------------------------------------------*/
    /**
     * @brief Type alias for the RX event callback signature (for ISR context).
     * @return true if the data was consumed and the RX buffer should switch.
     */
    using rx_event_func = std::function<bool(
        uint8_t *p, uint16_t size, BaseType_t xHigherPriorityTaskWoken)>;

    /**
     * @brief Structure to store registered RX callbacks with an owner ID.
     */
    typedef struct rx_event_callback_t
    {
        uint32_t owner;
        rx_event_func func;
    } rx_event_callback_t;

    /**
     * @brief Internal state flags (bit-field) for tracking driver status.
     */
    typedef struct state_t
    {
        volatile uint8_t init_flag     : 1;
        volatile uint8_t tx_busy       : 1;
        volatile uint8_t tx_timeout    : 1;
        volatile uint8_t rx_dma_enable : 1;
        volatile uint8_t rx_busy       : 1;
        volatile uint8_t rx_error      : 1;
    } state_t;

  public:
    /* Public Methods - Initialization and De-initialization -------------------*/
    explicit uart_drv_t(UART_HandleTypeDef *huart, uint16_t buf_length);
    ~uart_drv_t();
    status_t reset();

    /* Public Methods - Transmission -------------------------------------------*/
    status_t write(const uint8_t *p, uint16_t size, uint32_t waittime); // Polling
    status_t write(const uint8_t *p, uint16_t size);                   // DMA

    /* Public Methods - Reception Control --------------------------------------*/
    status_t enable_rx_dma();
    status_t disable_rx_dma();

    /* Public Methods - Custom Callback Management -----------------------------*/
    void add_rx_event_callback(const rx_event_func &func, uint32_t owner);
    status_t remove_rx_event_callback(uint32_t);

    /* Public Methods - HAL Callback Registration ------------------------------*/
    status_t register_event_callback(pUART_RxEventCallbackTypeDef pCallback);
    status_t unregister_event_callback();
    status_t register_callback(HAL_UART_CallbackIDTypeDef CB_ID,
                               pUART_CallbackTypeDef pCallback);
    status_t unregister_callback(HAL_UART_CallbackIDTypeDef CB_ID);

    /* Public Members - Internal State/Data ------------------------------------*/
    std::vector<rx_event_callback_t> rx_event_callbacks;
    static std::map<UART_HandleTypeDef *, uart_drv_t *> &uart_map();
    uint8_t *rx_buf[2];      // Double buffers for DMA reception
    uint8_t rx_buf_switch{}; // Index of the currently active buffer
    state_t state{};

  private:
    /* Private Members ---------------------------------------------------------*/
    UART_HandleTypeDef *_huart; // HAL handle for the peripheral
    uint16_t _rx_buf_size{};    // Size of each RX buffer
};

} // namespace pyro


#endif