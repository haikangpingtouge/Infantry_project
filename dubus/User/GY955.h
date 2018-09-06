#ifndef __GY955_H
#define __GY955_H

#include "base_class.h"
/* ----------------- 陀螺仪结构体 -------------------- */
typedef struct 
{
	int16_t  		Gyr_X;
	int16_t  		Gyr_y;
	int16_t  		Gyr_z;

	float     		Yaw;
	float			Roll;
	float 			Pitch;
	uint16_t       counter; 
	
}GY955_Class;

void Gy955ClassDataInit(void);
void AnalysisGyro(void);

#define GY955BACKLEN   1
#define GY955LEN   18
#define EULERANGLE 0x08
#define GYRO       0x04

#ifdef  __GB955_GLOBALS
#define __GB955_EXT
#else
#define __GB955_EXT extern
#endif


__GB955_EXT uint8_t GB955Buffer[GY955BACKLEN + GY955LEN];				//DBUS,DMA接收缓存
uint8_t GyroCRCCheck(void);
#endif /*__GY955_H*/
