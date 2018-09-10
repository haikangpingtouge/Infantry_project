#ifndef __PANTILTZOOM_H
#define __PANTILTZOOM_H
#include "Driver_DBUS.h"
#include  "GY955.h"
// #define PTZ_DATA_INIT  {{0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0,0,0,0}}  //��̨���ݳ�ʼ��


//��̨��
typedef struct
{
/* ----------------- ������̨����ṹ�� -------------------- */
   	motor Pitch;
	motor Yaw; 
	PIDCLASS  PitchSpeel;

	GY955_Class* Gyc;

/* ----------------- ������̨�������� -------------------- */
	int16_t P; //ң�����ݻ���
	int16_t Y;
	int16_t motor_id;

/* ----------------- �����Ա���� -------------------- */
	void (*PIDConnector)(uint8_t mode,float p,float i,float d);
}Pantiltzoom_Struct;


#define PITCH   0
#define YAW     3

#define INFANTRY4CEN_YAW        3960     //�ĺŲ�����̨����ֵ
#define INFANTRY4CEN_PITCH        2348  //�ĺŲ�����̨����ֵ
#define INFANTRY4_YAW_LEFT           4600 //�ĺŲ�����̨������ֵ
#define INFATNRY4_YAW_RIGHT          3300    //�ĺŲ�����̨�ұ����ֵ
#define INFANTRY4_PITCH_T           2610 //�ĺŲ�����̨������ֵ
#define INFATNRY4_PITCH_B          2260    //�ĺŲ�����̨�ұ����ֵ

#define INFANTRY1CEN_YAW        4266     //�ĺŲ�����̨����ֵ
#define INFANTRY1CEN_PITCH        3700  //�ĺŲ�����̨����ֵ
#define INFANTRY1_YAW_LEFT           4600 //�ĺŲ�����̨������ֵ
#define INFATNRY1_YAW_RIGHT          3300    //�ĺŲ�����̨�ұ����ֵ

void PTZ_Data_Analysis(motor* RM6623,uint8_t *Data);
void PanTiltZoom_Init(void);
//void PTZ_PID_param(uint8_t way);
//uint8_t PanTiltZoomCanTX(CAN_HandleTypeDef* hcan);
void PantiltzoomDbusDataConverter(void);
//int16_t LocalDeadLock(int16_t data,int16_t max,int16_t min);
static void PIDConnector(uint8_t mode,float p,float i,float d);
uint8_t RuneController(uint8_t *groal);
#endif // !__PANTILTZOOM_H
