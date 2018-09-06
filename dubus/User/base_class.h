#ifndef __BASE_CLASS_H
#define __BASE_CLASS_H

#include "stm32f4xx_hal.h"
#define FIRST_POWER_UP 1

/*-------------限幅结构体------------*/
typedef struct
{
    int16_t max;
    int16_t min;
}LIMIT;


/*-------------电机父类结构体------------*/
typedef struct
{
    int16_t 		    target;             //电机目标速度
    int16_t 		    real;               //电机实际速度
    int16_t 		    RealCurrent;            //实际电流
    int16_t         Real_Angle;             //;实际角度
    int16_t         pid_out;  
    float           Kp;
    float           Ki;
    float           Kd;
    float           Bise;
	float           Last_Bise; 
    float           Integral_bias; 
	float             before_last_bias;
	LIMIT*          limit;   
	// uint16_t    FrameRate;        			//帧率
	uint16_t       counter;     			//帧率计数器


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
#define L_Motor_Id ((uint16_t)0x200)//1到4电机
#define H_Motor_Id ((uint16_t)0x1FF)//5到8电机 



#endif
