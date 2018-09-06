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



#endif
