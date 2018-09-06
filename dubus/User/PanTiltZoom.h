#ifndef __PANTILTZOOM_H
#define __PANTILTZOOM_H
#include "Driver_DBUS.h"
#include  "GY955.h"
// #define PTZ_DATA_INIT  {{0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0}}  //云台数据初始化


//云台类
typedef struct
{
/* ----------------- 声明云台两轴结构体 -------------------- */
   	motor Pitch;
	motor Yaw; 
	PIDCLASS  PitchSpeel;

	GY955_Class* Gyc;

/* ----------------- 定义云台公有数据 -------------------- */
	int16_t P; //遥控数据缓存
	int16_t Y;
	int16_t motor_id;

/* ----------------- 定义成员函数 -------------------- */
	void (*PIDConnector)(uint8_t mode,float p,float i,float d);
}Pantiltzoom_Struct;


#define PITCH   0
#define YAW     3

void PTZ_Data_Analysis(motor* RM6623,uint8_t *Data);
void PanTiltZoom_Init(void);
//void PTZ_PID_param(uint8_t way);
//uint8_t PanTiltZoomCanTX(CAN_HandleTypeDef* hcan);
void PantiltzoomDbusDataConverter(void);
//int16_t LocalDeadLock(int16_t data,int16_t max,int16_t min);
static void PIDConnector(uint8_t mode,float p,float i,float d);
uint8_t RuneController(uint8_t *groal);
#endif // !__PANTILTZOOM_H
