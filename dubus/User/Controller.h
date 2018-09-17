#ifndef __CONTROLLER_H
#define __CONTROLLER_H
#include "Chassis.h"
#include "PanTiltZoom.h"


typedef enum{
	Location_PID = 0U,
	Incremental_PID = 1U
}_PID;
void PID_control(motor *motor,uint8_t PID);
void Limiter(int16_t *parameter,int16_t max,int16_t min);
float myabs(float a);

void OutPIDControl(int16_t target,int16_t real,Pantiltzoom_Struct* Ps);

//�޷�����������base_class.h�ļ���;
//����ֵ����������base_class.h�ļ���;

int16_t ExpertPIDControl(motor *motor);

#endif // ! __ONTROLLER_H
