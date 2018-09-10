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

#define INFANTRY4CEN_YAW        3960     //四号步兵云台中心值
#define INFANTRY4CEN_PITCH        2348  //四号步兵云台中心值
#define INFANTRY4_YAW_LEFT           4600 //四号步兵云台左边最大值
#define INFATNRY4_YAW_RIGHT          3300    //四号步兵云台右边最大值
#define INFANTRY4_PITCH_T           2610 //四号步兵云台左边最大值
#define INFATNRY4_PITCH_B          2260    //四号步兵云台右边最大值

#define INFANTRY1CEN_YAW        4266     //四号步兵云台中心值
#define INFANTRY1CEN_PITCH        3700  //四号步兵云台中心值
#define INFANTRY1_YAW_LEFT           4600 //四号步兵云台左边最大值
#define INFATNRY1_YAW_RIGHT          3300    //四号步兵云台右边最大值

void PTZ_Data_Analysis(motor* RM6623,uint8_t *Data);
void PanTiltZoom_Init(void);
//void PTZ_PID_param(uint8_t way);
//uint8_t PanTiltZoomCanTX(CAN_HandleTypeDef* hcan);
void PantiltzoomDbusDataConverter(void);
//int16_t LocalDeadLock(int16_t data,int16_t max,int16_t min);
static void PIDConnector(uint8_t mode,float p,float i,float d);
uint8_t RuneController(uint8_t *groal);
#endif // !__PANTILTZOOM_H
