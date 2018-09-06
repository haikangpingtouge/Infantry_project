#define __GB955_GLOBALS
#include  "GY955.h"
#include "Driver_DBUS.h"
GY955_Class Gyc;
uint8_t sum,i;
/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void Gy955ClassDataInit(void)
{
			Gyc.Gyr_X=0;
			Gyc.Gyr_y=0;
			Gyc.Gyr_z=0;

	 		Gyc.Yaw=0;
			Gyc.Roll=0;
			Gyc.Pitch=0;

	      Gyc.counter=0; 
}
/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void AnalysisGyro(void)
{
	if ((GB955Buffer[0]==0x5A)&&(GB955Buffer[1]==0x5A))
	 {
	 
		for (sum=0,i = 0; i < GB955Buffer[3]+4; i++)  sum+=GB955Buffer[i];
	
		if(GB955Buffer[i]==sum)//校验和判断
		{
			if((GB955Buffer[2]&GYRO)==GYRO)
			 {
				Gyc.Gyr_X=((GB955Buffer[4]<<8)|GB955Buffer[5]);
				Gyc.Gyr_y=((GB955Buffer[6]<<8)|GB955Buffer[7]);
				Gyc.Gyr_z=((GB955Buffer[8]<<8)|GB955Buffer[9]);
			 }
			if ((GB955Buffer[2]&EULERANGLE)==EULERANGLE)
			 {
				Gyc.Yaw=(uint16_t)((GB955Buffer[10]<<8)|GB955Buffer[11])/100.0f;
				Gyc.Roll=(int16_t)((GB955Buffer[12]<<8)|GB955Buffer[13])/100.0f;
				Gyc.Pitch=(int16_t)((GB955Buffer[14]<<8)|GB955Buffer[15])/100.0f;
			 }
		
			Gyc.counter++;

		}
	 }
}
/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  陀螺仪crc校验
	* @param  void
	* @retval sum  前面全部累加值
	**/
/* -------------------------------- end -------------------------------- */
uint8_t GyroCRCCheck(void)
{
	

	return sum;
}
