#include "pyro_trigger_drv.h"
#include "cmsis_os.h"

#define BLOCK_THRESHOLD 400
#define BLOCK_SPEED 0.3f
#define BLOCK_TIME 10

namespace pyro
{

trigger_drv_t::trigger_drv_t(motor_base_t *motor_base,
                     const pid_ctrl_t &_rotate_pid, 
                     const pid_ctrl_t &position_pid
                     )
    : motor_base(motor_base),
      _rotate_pid(_rotate_pid),
      _position_pid(position_pid)
{
}

void trigger_drv_t::set_gear_ratio(float gear_ratio)
{
    _gear_ratio = gear_ratio;
}

void trigger_drv_t::step_fire()
{
    set_
}

void trigger_drv_t::continue_fire()
{

    if(_current_rotate <= BLOCK_SPEED)
    {
    	_block_count1 ++;
    	if(_block_count1 > BLOCK_THRESHOLD)
    	{
            motor_base->send_torque(0.0f);
    		_block_count1 = 0;
    		vTaskDelay(BLOCK_TIME);
    		if(_current_rotate <= BLOCK_SPEED)
    		{
    			_block_count2 ++;
    			if(_block_count2 > BLOCK_THRESHOLD)
    			{
    				Step_Backward(shoot_con->shoot->my_trigger);
    				Step_Backward(shoot_con->shoot->my_trigger);
    				block_count2 = 0;
    				vTaskDelay(BLOCK_TIME);
    			}
    		}		
    	}
    }

    float torque_cmd = _rotate_pid.compute(_target_rotate,
                    _current_rotate, 0.001f);
    motor_base->send_torque(torque_cmd);
}

void trigger_drv_t::zero_force()
{
    float torque_cmd = _rotate_pid.compute(0.0f,
                    _current_rotate, 0.001f);
    motor_base->send_torque(torque_cmd);
}

void trigger_drv_t::set_rotate(float target_rotate)
{
    _target_rotate = target_rotate;
}

void trigger_drv_t::update_feedback()
{
    motor_base->update_feedback();
    _current_rotate = motor_base->get_current_rotate() / _gear_ratio;
}

}