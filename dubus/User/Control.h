#ifndef __CONTROL_H
#define __CONTROL_H 
#include "User_can.h"
#include "Controller.h"




typedef struct 
{
	Chassis_Struct*									ConCs;
	Pantiltzoom_Struct*             ConPs;
}BASE;
//#define CHASSISDATA      ChassisCanTX 
//#define PANTILTZOONDATA  PanTiltZoomCanTX

void Chassis_Control(Chassis_Struct* _Chassis,CAN_HandleTypeDef* hcan_1);
void Parsing_Motor_datas(BASE* This,  Can_Base_Class* Cbc);
void PanTiltZoom_Control(Pantiltzoom_Struct* This,CAN_HandleTypeDef* hcan);
void Control_Chassis_PI(Chassis_Struct* _Chassis);
void Control_PTZ_PID(Pantiltzoom_Struct* This);
void DUBS_Data_RX(BASE* This,DBUSDecoding_Type *data);
void CAN_TRANSMIT(CAN_HandleTypeDef* hcan,int STD_ID,int Letter0,int Letter1,int Letter2,int Letter3);

void FRune(void);
void CHUANKOU(void);

extern float lowbuffer;
#endif // !CONTROL_H
