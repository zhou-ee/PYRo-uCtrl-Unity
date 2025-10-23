#ifndef __PYRO_MACUNUM_CHASSIS_DRV_H__
#define __PYRO_MACUNUM_CHASSIS_DRV_H__

#include "pyro_chassis_base_drv.h"
#include "pyro_wheel_drv.h"


namespace pyro
{
class macunum_chassis_drv_t : public chassis_base_drv_t
{
    public:
        enum macunum_type
        {
            TYPE_X,
            TYPE_O,
        };
        macunum_chassis_drv_t(wheel_drv_t* wheel_1, wheel_drv_t* wheel_2, wheel_drv_t* wheel_3, wheel_drv_t* wheel_4, macunum_type type ,float _rx,float _ry);
        void control(float dt) override;
        void set_target(float vx, float vy, float wz) override;
        void update() override;
        void zero_force() override;
    private:
        wheel_drv_t* _wheel_1,* _wheel_2,* _wheel_3,* _wheel_4;//lf,rf,rb,lb,counter clockwise
        float _target_vx,_target_vy,_target_wz;
        float _feedback_vx,_feedback_vy,_feedback_wz;
        macunum_type _type;

        // x + is forward while y - is left
        float _rx; // the x offset of the number one wheel
        float _ry; // the y offset of the number one wheel

        float _wheel_spd[4];

};
};
#endif