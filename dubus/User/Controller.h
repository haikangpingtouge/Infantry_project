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
void usart2_send_char(uint8_t c,UART_HandleTypeDef* huart);
void usart2_niming_report(uint8_t fun,uint8_t*data,uint8_t len,UART_HandleTypeDef* huart);
void PID_Debug(int16_t Target,int16_t Real,UART_HandleTypeDef* huart);
void OutPIDControl(int16_t target,int16_t real,Pantiltzoom_Struct* Ps);

//限幅函数定义在base_class.h文件中;
//绝对值函数定义在base_class.h文件中;

int16_t ExpertPIDControl(motor *motor);
#endif // ! __ONTROLLER_H
