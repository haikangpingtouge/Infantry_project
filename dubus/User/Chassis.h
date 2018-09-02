#ifndef __CHASSIS_H
#define __CHASSIS_H
#include "Driver_DBUS.h"
typedef struct
{
/* ----------------- �����������ֽṹ�� -------------------- */
    motor chassis_LF;
    motor chassis_RF;
    motor chassis_LB;
    motor chassis_RB;
	//   motor *PTZ_yaw;
	//   motor *PTZ_pitch;
/* ----------------- ������̹������� -------------------- */
    int16_t Target_Vx; //ң�����ݻ���
    int16_t Target_Vy;//ң�����ݻ���
    float Target_Wt;//ң�����ݻ���
    int16_t  motor_id;
	  int16_t max;
	  int16_t min;

/* ----------------- �����Ա���� -------------------- */

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
