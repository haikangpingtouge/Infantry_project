#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "Driver_DBUS.h"
typedef struct
{
/* ----------------- 声明底盘四轮结构体 -------------------- */
    motor chassis_LF;
    motor chassis_RF;
    motor chassis_LB;
    motor chassis_RB;
	//   motor *PTZ_yaw;
	//   motor *PTZ_pitch;
/* ----------------- 定义底盘公有数据 -------------------- */
    int16_t Target_Vx; //遥控数据缓存
    int16_t Target_Vy;//遥控数据缓存
    float Target_Wt;//遥控数据缓存
    int16_t  motor_id;
	  int16_t max;
	  int16_t min;

/* ----------------- 定义成员函数 -------------------- */

    void (*PIDConnector)(uint8_t mode,float p,float i,float d);

}Chassis_Struct;


#define MaxWheelSpeed                   2700
#define LF       0
#define RF       3
#define LB       6
#define RB       9

// void chassis_motordata_init(void);
void Chassis_Data_Analysis(motor* RM3508,uint8_t *Data);
void MW_TheCalculationSpeed(float Vx, float Vy, float Wt,Chassis_Struct* D);
void Chassis_init(void);
uint8_t ChassisCanTX(CAN_HandleTypeDef* hcan_1);


 static void PIDConnector(uint8_t mode,float p,float i,float d);

#endif
