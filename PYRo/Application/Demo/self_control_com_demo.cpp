#include "pyro_core_config.h"
#include "cmsis_os.h"
#include "task.h"
#include "message_buffer.h"

#include "pyro_uart_drv.h"
extern "C"
{

float recv_axis1_angle = 0.0f;
float recv_axis2_angle = 0.0f;
float recv_axis3_angle = 0.0f;
float recv_axis4_angle = 0.0f;
float recv_axis5_angle = 0.0f;
float recv_axis6_angle = 0.0f;
namespace pyro
{
class self_control_com
{
    public:
    MessageBufferHandle_t _self_control_msg_buffer{};
    uart_drv_t *_self_control_uart;
    self_control_com(pyro::uart_drv_t *self_control_uart):_self_control_uart(self_control_uart)
    {

    }

    void init()
    {
       _self_control_msg_buffer = xMessageBufferCreate(108);
       _self_control_uart->add_rx_event_callback(
        [this](uint8_t *buf, uint16_t len,
               BaseType_t xHigherPriorityTaskWoken) -> bool
        { return self_control_callback(buf, len, xHigherPriorityTaskWoken); },
        reinterpret_cast<uint32_t>(this));
    }

    bool self_control_callback(uint8_t *buf, uint16_t len,
                             BaseType_t xHigherPriorityTaskWoken)
    {
        if (len == 32)
        {
            // Check if the protocol is higher priority than any other active RC
            // protocol
            
            xMessageBufferSendFromISR(_self_control_msg_buffer, buf, len,
                                        &xHigherPriorityTaskWoken);
                return true;
            
        }
        return false;
    }
    void unpack(uint8_t* buf )
    {
        recv_axis1_angle = *((float*)(buf+7));
        recv_axis2_angle = *((float*)(buf+11));
        recv_axis3_angle = *((float*)(buf+15));
        recv_axis4_angle = *((float*)(buf+19));
        recv_axis5_angle = *((float*)(buf+23));
    }

    void thread()
    {
        static uint8_t self_control_buf[32];
        static size_t xReceivedBytes;
        while(true)
        {
            xReceivedBytes = xMessageBufferReceive(_self_control_msg_buffer, self_control_buf,
                                               32, 100);
            if (xReceivedBytes == 32)
            {
                unpack(self_control_buf); // Process the packet
            }
            vTaskDelay(5);
        }
    }
};

}
pyro::self_control_com * _self_control_com;
void self_control_com_task(void *argument)
{
    _self_control_com = new pyro::self_control_com(pyro::uart_drv_t::get_instance(pyro::uart_drv_t::uart7));
    _self_control_com->init();
    _self_control_com->thread();
}

}