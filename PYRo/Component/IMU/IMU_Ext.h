#ifndef __IMU_EXT_H__
#define __IMU_EXT_H__

#include "main.h"
#include "IMU_Base.h"

IMU_obj* IMU_Ext_Type1_Factory(void);
void IMU_Ext_Type1_Update(IMU_obj* obj);

#endif