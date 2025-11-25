#include "pyro_vofa.h"

#include "pyro_core_dma_heap.h"
#include "task.h"

#include "cstring"

#include "referee.h"
#include "pyro_powermeter.h"
#include "pyro_chassis_drv.h"


extern pyro::powermeter_data power_data;
extern pyro::wheel_drv_t *wheel_drv_4;
extern pyro::wheel_drv_t *wheel_drv_3;
extern pyro::wheel_drv_t *wheel_drv_2;
extern pyro::wheel_drv_t *wheel_drv_1;
extern pyro::chassis_drv_t *chassis_drv;
extern float raw_torque;


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
    static vofa_drv_t instance(max_length, uart_drv_t::get_instance(uart_drv_t::uart10));
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
        for (uint8_t i = 0; i < size; ++i)
        {
            _data_pack[offset++] = data[i];
        }
    }
    _data_pack[offset] = *reinterpret_cast<float *>(frame_tail);
}

void vofa_drv_t::send()
{
    _vofa_uart->write(reinterpret_cast<uint8_t *>(_data_pack),
                      (_length + 1) * 4);
}

void vofa_drv_t::thread()
{
    
    float buffer_energy = static_cast<float>(referee_data.power_heat.buffer_energy);
    add_data(&buffer_energy);

    // float *real_power = &power_data.power;
    // add_data(real_power);

    float *predict_power_4 = &chassis_drv->_wheel_data.at(3).power_predict;
    add_data(predict_power_4);

    // float *torque_4 = &wheel_drv_4->torque_cmd;
    // add_data(torque_4);

    // float *torque_without_predict = &raw_torque;
    // add_data(torque_without_predict);

    // float *rotate = &wheel_drv_4->rotate;
    // add_data(rotate);


    // float *predict_power_1 = &wheel_drv_1->power_control_drv.power_predict;
    // add_data(predict_power_1);

    // float *predict_power_2 = &wheel_drv_2->power_control_drv.power_predict;
    // add_data(predict_power_2);

    // float *predict_power_3 = &wheel_drv_3->power_control_drv.power_predict;
    // add_data(predict_power_3);

    // float *torque_1 = &wheel_drv_1->torque_cmd;
    // add_data(torque_1);

    // float *torque_2 = &wheel_drv_2->torque_cmd;
    // add_data(torque_2);

    // float *torque_3 = &wheel_drv_3->torque_cmd;
    // add_data(torque_3);

    // float *tau = &wheel_drv_4->torque_cmd;
    // add_data(tau);

    // float *current = &wheel_drv_2->_current_speed;
    // add_data(current);

    // float *target = &wheel_drv_2->_target_speed;
    // add_data(target);

    while (true)
    {
        buffer_energy = static_cast<float>(referee_data.power_heat.buffer_energy);
        update_data();
        send();
        vTaskDelay(10);
    }
}

} // namespace pyro

extern "C" void pyro_vofa_task(void *arg)
{
    pyro::vofa_drv_t &vofa = pyro::vofa_drv_t::get_instance(15);
    vofa.thread();
}