#ifndef __DATA_PRTOCOL_H
#define __DATA_PRTOCOL_H
 
 #include "base_class.h"
extern uint8_t DataScope_OutPut_Buffer[42];	   //������֡���ݻ�����
extern uint8_t qq;
void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
void DataScope_Get_Channel_Data(float Data,unsigned char Channel);    // дͨ�������� ������֡���ݻ�����

unsigned char DataScope_Data_Generate(unsigned char Channel_Number);  // ����֡�������ɺ��� 
void MiniBalanceDebug(float Data1,float Data2,UART_HandleTypeDef* huart);
//void usart2_send_char(uint8_t c,UART_HandleTypeDef* huart);
void flagDebug(float data1,float data2,UART_HandleTypeDef* huart);
 
#endif 



