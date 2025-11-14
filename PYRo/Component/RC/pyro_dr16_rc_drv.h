/**
 * @file pyro_dr16_rc_drv.h
 * @brief Header file for the PYRO DR16 Remote Control Driver.
 *
 * This file defines the `pyro::dr16_drv_t` class, which implements the
 * protocol-specific logic for decoding the data packets from a DJI DR16
 * receiver (used with remote controllers like the RoboMaster/DJI FPV).
 * It inherits from the generic `rc_drv_t` base class.
 *
 * @author Lucky
 * @version 1.0.0
 * @date 2025-10-09
 * @copyright [Copyright Information Here]
 */

#ifndef __PYRO_DR16_RC_DRV_H__
#define __PYRO_DR16_RC_DRV_H__

/* Includes ------------------------------------------------------------------*/
#include "pyro_rc_base_drv.h"

/* Defines -------------------------------------------------------------------*/


namespace pyro
{

/* Class Definition ----------------------------------------------------------*/
/**
 * @brief Driver class for the DJI DR16 remote control protocol.
 *
 * Implements the concrete logic for handling DR16 data packets, including
 * unpacking channels, mouse/keyboard inputs, error checking, and signaling
 * the main processing thread.
 */
class dr16_drv_t : public rc_drv_t
{
    /* Private Types - Raw Buffer --------------------------------------------*/
    /**
     * @brief Raw structure of the 18-byte DR16 data packet.
     *
     * Uses bit-fields to extract 11-bit channel data and 2-bit switch data.
     * The `__packed` attribute is necessary for direct memory mapping.
     */
    typedef struct __packed
    {
        uint32_t ch0 : 11; // X1 (Right Stick H)
        uint32_t ch1 : 11; // Y1 (Right Stick V)
        uint32_t ch2 : 11; // X2 (Left Stick H)
        uint32_t ch3 : 11; // Y2 (Left Stick V)
        uint32_t s1  : 2;  // Switch 1
        uint32_t s2  : 2;  // Switch 2

        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        uint8_t press_l;
        uint8_t press_r;
        uint16_t key_code;
        uint16_t wheel;

    } dr16_buf_t;



    /* Private Types - Control Data ------------------------------------------*/
    /**
     * @brief Decoded structure for consumer use.
     *
     * Contains separated and scaled data for RC, mouse, and keyboard inputs.
     */

    /**
     * @brief Type alias for the data output callback function signature.
     */

  public:
    enum dr16_sw_state_t
    {
        DR16_SW_UP   = 1,
        DR16_SW_MID  = 3,
        DR16_SW_DOWN = 2
    };
    enum dr16_sw_ctrl_t
    {
        DR16_SW_NO_CHANGE   = 0,
        DR16_SW_UP_TO_MID   = 1,
        DR16_SW_MID_TO_DOWN = 2,
        DR16_SW_DOWN_TO_MID = 3,
        DR16_SW_MID_TO_UP   = 4,
    };

    enum dr16_sw_pos_t
    {
        DR16_SW_RIGHT = 0,
        DR16_SW_LEFT  = 1,
    };
    enum dr16_channel_t
    {
        DR16_CH_RIGHT_X = 0,
        DR16_CH_RIGHT_Y = 1,
        DR16_CH_LEFT_X  = 2,
        DR16_CH_LEFT_Y  = 3,
    };
    enum key_ctrl_t
    {
        KEY_RELEASED = 0,
        KEY_PRESSED  = 1,
        KEY_HOLD     = 2
    };
    typedef struct key_t
    {
        uint8_t ctrl;
        uint32_t time;
    } key_t;
    typedef struct dr16_switch_t
    {
        uint8_t state;
        uint8_t ctrl;
    } dr16_switch_t;
    typedef struct dr16_ctrl_t
    {
        struct
        {
            float ch[4];        ///< Channel values scaled to [-1.0, 1.0]
            dr16_switch_t s[2]; ///< Switch positions
            float wheel;        ///< Wheel value scaled to [-1.0, 1.0]
        } rc;
        struct
        {
            float x;
            float y;
            float z;
            key_t press_l; ///< Left mouse button state
            key_t press_r; ///< Right mouse button state
        } mouse;
        struct
        {
            key_t w;
            key_t s;
            key_t a;
            key_t d;
            key_t shift;
            key_t ctrl;
            key_t q;
            key_t e;
            key_t r;
            key_t f;
            key_t g;
            key_t z;
            key_t x;
            key_t c;
            key_t v;
            key_t b;
        } key; ///< Keyboard key states
    } dr16_ctrl_t;
    /* Public Members --------------------------------------------------------*/

    /* Public Methods - Construction and Lifecycle (Override)
     * ------------------*/
    explicit dr16_drv_t(uart_drv_t *dr16_uart);
    status_t init() override;
    void enable() override;
    void disable() override;
    void thread() override;

    /* Public Methods - Configuration
     * ------------------------------------------*/

    void config_rc_cmd(const cmd_func &func) override;

  private:
    dr16_ctrl_t _dr16_ctrl{}; ///< The latest decoded control data.
    dr16_ctrl_t _dr16_last_ctrl{};
    /* Private Methods - Overrides
     * ---------------------------------------------*/
    /**
     * @brief The callback registered with the UART driver (ISR context).
     *
     * Receives raw UART data and forwards it to the FreeRTOS message buffer.
     */
    bool rc_callback(uint8_t *buf, uint16_t len,
                     BaseType_t xHigherPriorityTaskWoken) override;

    /* Private Methods - Processing
     * --------------------------------------------*/
    /**
     * @brief Performs range checking on raw DR16 channel data.
     * @param dr16_buf Pointer to the raw data buffer.
     * @return PYRO_OK if data is valid, PYRO_ERROR otherwise.
     */
    static status_t error_check(const dr16_buf_t *dr16_buf);
    /**
     * @brief Checks for switch state changes (e.g., UP_TO_MID).
     * @param dr16_switch The switch state object (to be updated).
     * @param state The new raw state from the receiver.
     */
    static void check_ctrl(dr16_switch_t &dr16_switch, uint8_t state);
    /**
     * @brief Checks for key state changes (PRESSED, HOLD, RELEASED).
     * @param key The key state object (to be updated).
     * @param state The new raw state (0 or 1) from the receiver.
     */
    static void check_ctrl(key_t &key, uint8_t state);
    /**
     * @brief Unpacks raw DR16 data into the `dr16_ctrl_t` structure.
     * @param dr16_buf Pointer to the raw data buffer to unpack.
     */
    void unpack(const dr16_buf_t *dr16_buf);
};

} // namespace pyro
#endif