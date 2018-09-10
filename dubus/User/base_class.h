#ifndef __BASE_CLASS_H
#define __BASE_CLASS_H

#include "stm32f4xx_hal.h"
#define FIRST_POWER_UP 1

/*-------------�޷��ṹ��------------*/
typedef struct
{
    int16_t max;
    int16_t min;
}LIMIT;


/*-------------�������ṹ��------------*/
typedef struct
{
    int16_t 		    target;             //���Ŀ���ٶ�
    int16_t 		    real;               //���ʵ���ٶ�
    int16_t 		    RealCurrent;            //ʵ�ʵ���
    int16_t         Real_Angle;             //;ʵ�ʽǶ�
    int16_t         pid_out;  
    float           Kp;
    float           Ki;
    float           Kd;
    float           Bise;
	float           Last_Bise; 
    float           Integral_bias; 
	float             before_last_bias;
	LIMIT*          limit;   
	// uint16_t    FrameRate;        			//֡��
	uint16_t       counter;     			//֡�ʼ�����


}motor;

typedef struct
{
	float           P_out;
	float           I_out;
	float           D_out;
	int16_t         pid_out; 
	float        	 Kp;
	float       	Ki;
	float   		Kd;
	float           Bise;
	float           Last_Bise; 
	float           Integral_bias;
	float           before_last_bias;
}PIDCLASS;




typedef struct
{
	uint32_t aa;
	uint32_t bb;


}Time_Counter;
#define L_Motor_Id ((uint16_t)0x200)//1��4���
#define H_Motor_Id ((uint16_t)0x1FF)//5��8��� 

uint8_t LimiterUint8(uint8_t parameter,uint8_t max,uint8_t min);
uint16_t LimiterUint16(uint16_t parameter,uint16_t max,uint16_t min);
uint32_t LimiterUint32(uint32_t parameter,uint32_t max,uint32_t min);
int16_t LimiterInt16(int16_t parameter,int16_t max,int16_t min);
int32_t LimiterInt32(int32_t parameter,int32_t max,int32_t min);
int LimiterInt(int parameter,int max,int min);
float LimiterFloat(float parameter,float max,float min);


int16_t absInt16_t(int16_t a);
int32_t absInt32_t(int32_t a);
int absInt(int a);
float absFloat(float a);


#endif
