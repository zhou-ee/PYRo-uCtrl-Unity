#include "pyro_vofa.h"

#include "pyro_core_dma_heap.h"
#include "task.h"

#include "cstring"

namespace pyro
{
vofa_drv_t::vofa_drv_t(uint8_t max_length, uart_drv_t *uart)
{
    _data_pack = static_cast<float *>(pvPortDmaMalloc(4 * max_length));
    _length    = 0;
    _vofa_uart = uart;
}

vofa_drv_t::~vofa_drv_t()
{
    if (_data_pack)
    {
        delete[] _data_pack;
        _data_pack = nullptr;
    }
}

vofa_drv_t &vofa_drv_t::get_instance(uint8_t max_length)
{
    static vofa_drv_t instance(max_length, &get_uart1());
    return instance;
}

void vofa_drv_t::init()
{
}

void vofa_drv_t::add_data(float *data)
{
    if (data)
    {
        data_node_t temp;
        temp.data = data;
        temp.size = 1;
        _length += temp.size;
        _data_nodes.push_back(temp);
    }
}

void vofa_drv_t::add_data(float *data, const uint8_t len)
{
    if (data)
    {
        data_node_t temp;
        temp.data = data;
        temp.size = len;
        _length += temp.size;
        _data_nodes.push_back(temp);
    }
}

void vofa_drv_t::remove_data(const float *data)
{
    for (auto it = _data_nodes.begin(); it != _data_nodes.end(); ++it)
    {
        if (it->data == data)
        {
            _length -= it->size;
            _data_nodes.erase(it);
            break;
        }
    }
}

void vofa_drv_t::update_data()
{
    static uint8_t frame_tail[4] = {0x00, 0x00, 0x80, 0x7F};
    uint8_t offset               = 0;
    for (const auto &[data, size] : _data_nodes)
    {
        memcpy(_data_pack + offset, data, size);
        offset += size;
    }
    memcpy(_data_pack + offset, frame_tail, 4);
}

void vofa_drv_t::send()
{
    _vofa_uart->write(reinterpret_cast<uint8_t *>(_data_pack),
                      (_length + 1) * 4);
}
float test_data[5] = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f};
void vofa_drv_t::thread()
{
    add_data(test_data, 5);
    while (true)
    {
        update_data();
        send();
        vTaskDelay(1);
    }
}

} // namespace pyro

extern "C" void pyro_vofa_demo(void *arg)
{
    pyro::vofa_drv_t &vofa = pyro::vofa_drv_t::get_instance(15);
    vofa.thread();
}