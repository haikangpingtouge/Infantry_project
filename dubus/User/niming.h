#ifndef __NIMING_H
#define __NIMING_H

#include "base_class.h"

typedef struct 
{
	void(*f)(float,float,UART_HandleTypeDef* huart);
	void(*i16)(int16_t,int16_t,UART_HandleTypeDef* huart);
}Niming_Class;
extern Niming_Class  Debug;

void NimingClassInit(void);
void usart2_send_char(uint8_t c,UART_HandleTypeDef* huart);
void usart2_niming_report(uint8_t fun,uint8_t*data,uint8_t len,UART_HandleTypeDef* huart);

void PID_Debugfloat(float Target,float Real,UART_HandleTypeDef* huart);
void PID_Debugint16_t(int16_t Target,int16_t Real,UART_HandleTypeDef* huart);

#endif	// __NIMING_H
