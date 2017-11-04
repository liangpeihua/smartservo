#ifndef __SERVO_PROTECT_H__
#define __SERVO_PROTECT_H__

#include "M051Series.h"
#include "mygpio.h"
#include "MePwm.h"
#include "dataflash.h"


typedef struct
{
    u8 Ready;
    volatile s32 TotalVoltage;	    //母线电压 batteryVoltage
    volatile s16 TotalCurrent;	    //总电流电流
    volatile s32 AverageSpeed;

    volatile s32 L_VehicleSpeed;	    //车辆速度 unit:m/H 
    volatile s32 R_VehicleSpeed;	    //车辆速度 unit:m/H 

    volatile u16 L_IQCurrent;		//IQ电流
    volatile u16 R_IQCurrent;		//IQ电流

    volatile s16 L_BusCurrent;       //母线电流
    volatile s16 R_BusCurrent;

    volatile s16 L_HallState;		//霍尔状态
    volatile s16 R_HallState;

    volatile s16 L_VectorVol;		    //向量电压 Fov vector voltage
    volatile s16 R_VectorVol;		    //向量电压 Fov vector voltage

}StrctSuperise;


#endif/* __SMARTSERVO_H__ */
