#include "PanTiltZoom.h"
Pantiltzoom_Struct Ps; //云台对象

LIMIT LtPit;   //纵轴继承  比较对象
LIMIT LtYaw;  //横轴继承    比较对象类
static float *pid_list[6];  //PID列表
extern GY955_Class Gyc;
// /* -------------------------------- begin -------------------------------- */
// /**
//   * @brief  构造函数
//   * @param  void
//   * @retval void
//  **/
// /* -------------------------------- end -------------------------------- */
// PTZ *NewPTZ(PTZ *this)
// {
    
//     return this;
// } 
            
		
		
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  云台类的初始化
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void PanTiltZoom_Init(void)
{
/*---------云台公有数据初始化-------------*/
    Ps.Pitch.Bise=0;
    Ps.Pitch.Integral_bias=0;           //云台PID列表初始化
    Ps.Pitch.Kp=-8;                      pid_list[0]=&Ps.Pitch.Kp; 
    Ps.Pitch.Ki=0;                      pid_list[1]=&Ps.Pitch.Ki;
    Ps.Pitch.Kd=-0.5;                      pid_list[2]=&Ps.Pitch.Kd;
    Ps.Pitch.Last_Bise=0;
    Ps.Pitch.pid_out=0;
    Ps.Pitch.real=0;
    Ps.Pitch.Real_Angle=0;
    Ps.Pitch.RealCurrent=0;
    Ps.Pitch.target=INFANTRY4CEN_PITCH;
    Ps.Pitch.limit=&LtPit;
	 Ps.Pitch.limit->max=700;
	Ps.Pitch.limit->min=-700;
	Ps.Pitch.counter= 0;

    Ps.Yaw.Bise=0;
    Ps.Yaw.Integral_bias=0;           //云台PID列表初始化
    Ps.Yaw.Kp=0;                        pid_list[3]=&Ps.Yaw.Kp;
    Ps.Yaw.Ki=0;                        pid_list[4]=&Ps.Yaw.Ki;
    Ps.Yaw.Kd=0;                        pid_list[5]=&Ps.Yaw.Kd;
    Ps.Yaw.Last_Bise=0;
    Ps.Yaw.pid_out=0;
    Ps.Yaw.real=0;
    Ps.Yaw.Real_Angle=0;
    Ps.Yaw.RealCurrent=0;
    Ps.Yaw.target=INFANTRY4CEN_YAW;
	  Ps.Yaw.limit=&LtYaw;
    Ps.Yaw.limit->max=5000;
	Ps.Yaw.limit->min=-5000;
	Ps.Yaw.counter= 0;

	Ps.PitchSpeel.pid_out=0;
	Ps.PitchSpeel.Kp=-0.4;
	Ps.PitchSpeel.Ki=0;
	Ps.PitchSpeel.Kd=-9.5;
	Ps.PitchSpeel.Bise=0;
	Ps.PitchSpeel.Last_Bise=0;
	Ps.PitchSpeel.Integral_bias=0;
	Ps.PitchSpeel.before_last_bias=0;
	Ps.PitchSpeel.P_out=0;
	Ps.PitchSpeel.I_out=0;
	Ps.PitchSpeel.D_out=0;

		
		Ps.P=0;
		Ps.Y=0;
		Ps.motor_id=0x1FF;

		/* ----------------- 继承陀螺仪 -------------------- */
		Ps.Gyc=&Gyc;

/* ----------------- 云台成员函数初始化 -------------------- */
    Ps.PIDConnector=PIDConnector;

}
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  云台数据解析
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void PTZ_Data_Analysis(motor* RM6623,uint8_t *Data)
{
	static int16_t last_real_angle,Real_Angle;
    Real_Angle=((int16_t)(Data[0]<<8)|Data[1]);
	RM6623->RealCurrent=((int16_t)(Data[4]<<8)|Data[5]);
	/* ----------------- 过零点处理 -------------------- */
	if (Real_Angle - last_real_angle < -6000 ) 
	{
		RM6623->counter++;
	}
	if (last_real_angle - Real_Angle <-6000)
	 {
		RM6623->counter--;
	 }
	 last_real_angle = Real_Angle;
	 RM6623->Real_Angle= Real_Angle + (8192*RM6623->counter);

}
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  云台pid不同模式的初始化参数
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
//void PTZ_PID_param(uint8_t way)
//{
//    switch(way)
//    {
//        case FIRST_POWER_UP:    //第一次上电初始化模式
//        Ps.Pitch.Kd=0;
//        Ps.Pitch.Ki=0;
//        Ps.Pitch.Kp=0;

//        Ps.Yaw.Kd=0;
//        Ps.Yaw.Ki=0;
//        Ps.Yaw.Kp=0;
//        break;

//    }

//}

///* -------------------------------- begin -------------------------------- */
//    /**
//    * @brief  
//    * @param  
//    * @retval 
//    **/
///* -------------------------------- end -------------------------------- */
//uint8_t PanTiltZoomCanTX(CAN_HandleTypeDef* hcan)
//{
//   if((Ps.motor_id != L_Motor_Id) && (Ps.motor_id != H_Motor_Id))   		//id范围是否在1到8
//   {
//       hcan->pTxMsg->StdId = 0x00; //0~0x7ff设置出错后id设置成无效，以防电机发生不可预料的事
//       return 0x01;
//   }
//    hcan->pTxMsg->StdId = Ps.motor_id; //0~0x7ff
//    hcan->pTxMsg->IDE=CAN_ID_STD;//选择标准id
//    hcan->pTxMsg->RTR=0x00;
//    hcan->pTxMsg->DLC=8;

//    hcan->pTxMsg->Data[0] = (Ps.Yaw.pid_out >> 8) & 0xFF;
//    hcan->pTxMsg->Data[1] = Ps.Yaw.pid_out & 0xFF;
//    hcan->pTxMsg->Data[2] = (Ps.Pitch.pid_out >> 8) & 0xFF;
//    hcan->pTxMsg->Data[3] = Ps.Pitch.pid_out & 0xFF;
//	  hcan->pTxMsg->Data[4] =0;
//	  hcan->pTxMsg->Data[5]=0;
//	  hcan->pTxMsg->Data[6]=0;
//	  hcan->pTxMsg->Data[7]=0;
//	 
//   if(HAL_CAN_Transmit(hcan, 1000)!=HAL_OK)
//	 {
//		 do{
//			 		if(hcan->Instance->ESR!=0)
//						{
//								hcan->Instance->MCR|=0x02;
//								hcan->Instance->MCR&=0xfd;
//						}
//		 }while(!(hcan->Instance->TSR&(7<<26)));//等待三个邮箱为空
//	 }
//   return 0x00;
//}
/* -------------------------------- begin -------------------------------- */
    /**
    * @brief  云台得到目标值的换算器
    * @param  
    * @retval 
    **/
/* -------------------------------- end -------------------------------- */
void PantiltzoomDbusDataConverter(void)
{
	Ps.Pitch.target=Ps.P+INFANTRY4CEN_PITCH;
	Ps.Yaw.target=Ps.Y+INFANTRY4CEN_YAW;
	Ps.Pitch.target = LimiterInt16(Ps.Pitch.target,INFANTRY4_PITCH_T,INFATNRY4_PITCH_B);
		Ps.Yaw.target = LimiterInt16(Ps.Yaw.target,INFANTRY4_YAW_LEFT,INFATNRY4_YAW_RIGHT);

}


///* -------------------------------- begin -------------------------------- */
//    /**
//    * @brief  
//    * @param  
//    * @retval 
//    **/
///* -------------------------------- end -------------------------------- */
//int16_t LocalDeadLock(int16_t data,int16_t max,int16_t min)
//{
////    if(Ps.Pitch.Real_Angle<3416)
//}


/* -------------------------------- begin -------------------------------- */
    /**
    * @brief  
    * @param  
    * @retval 
    **/
/* -------------------------------- end -------------------------------- */
static void PIDConnector(uint8_t mode,float p,float i,float d)
{
    *pid_list[mode]=p;
    *pid_list[mode+1]=i;
    *pid_list[mode+2]=d;
}


/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  自动打神符云台控制
	* @param  目标角度
	* @retval void
	**/
/* -------------------------------- end -------------------------------- */
uint8_t RuneController(uint8_t *groal)
{
	Ps.Pitch.target = 3700+((groal[0]-90)*22);
	Ps.Yaw.target = 4266+((groal[1]-90)*22); 
	
		Ps.Pitch.target=Ps.Pitch.target<3800?3800:Ps.Pitch.target;
	Ps.Pitch.target=Ps.Pitch.target>4300?4300:Ps.Pitch.target;
	
	Ps.Yaw.target=Ps.Yaw.target<3500?3500:Ps.Yaw.target;
	Ps.Yaw.target=Ps.Yaw.target>5100?5100:Ps.Yaw.target;
	

	return 0;
}
