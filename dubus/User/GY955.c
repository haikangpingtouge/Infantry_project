#include  "GY955.h"

/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void AnalysisGyro()
{
	static uint8_t i=0,sum;
	
		for(sum=0,i=0;i<(Cloud_Gyro_Data[3]+4);i++)
			sum+=Cloud_Gyro_Data[i];
		if(sum==Cloud_Gyro_Data[i])//Ð£ÑéºÍÅÐ¶Ï
		{
			
			CloudParam.Cloud_Gyro.Gyr_X=((Cloud_Gyro_Data[4]<<8)|Cloud_Gyro_Data[5]);
			CloudParam.Cloud_Gyro.Gyr_Y=((Cloud_Gyro_Data[6]<<8)|Cloud_Gyro_Data[7]);
			CloudParam.Cloud_Gyro.Gyr_Z=((Cloud_Gyro_Data[8]<<8)|Cloud_Gyro_Data[9]);
	
			CloudParam.Cloud_Gyro.Yaw=(uint16_t)((Cloud_Gyro_Data[10]<<8)|Cloud_Gyro_Data[11])/100.0f;
			CloudParam.Cloud_Gyro.Roll=(int16_t)((Cloud_Gyro_Data[12]<<8)|Cloud_Gyro_Data[13])/100.0f;
			CloudParam.Cloud_Gyro.Pitch=(int16_t)((Cloud_Gyro_Data[14]<<8)|Cloud_Gyro_Data[15])/100.0f;
			
			CloudParam.Cloud_Gyro.FrameRate++;
		}
}

