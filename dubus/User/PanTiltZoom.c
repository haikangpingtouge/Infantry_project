#include "PanTiltZoom.h"
Pantiltzoom_Struct Ps; //��̨����
LIMIT LtPit;   //����̳�  �Ƚ϶���
LIMIT LtYaw;  //����̳�    �Ƚ϶�����
static float *pid_list[6];  //PID�б�
// /* -------------------------------- begin -------------------------------- */
// /**
//   * @brief  ���캯��
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
  * @brief  ��̨��ĳ�ʼ��
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void PanTiltZoom_Init(void)
{
/*---------��̨�������ݳ�ʼ��-------------*/
    Ps.Pitch.Bise=0;
    Ps.Pitch.Integral_bias=0;           //��̨PID�б��ʼ��
    Ps.Pitch.Kp=0;                      pid_list[0]=&Ps.Pitch.Kp; 
    Ps.Pitch.Ki=0;                      pid_list[1]=&Ps.Pitch.Ki;
    Ps.Pitch.Kd=0;                      pid_list[2]=&Ps.Pitch.Kd;
    Ps.Pitch.Last_Bise=0;
    Ps.Pitch.pid_out=0;
    Ps.Pitch.real=0;
    Ps.Pitch.Real_Angle=0;
    Ps.Pitch.RealCurrent=0;
    Ps.Pitch.target=3700;
    Ps.Pitch.limit=&LtPit;
	 Ps.Pitch.limit->max=3000;
	Ps.Pitch.limit->min=-3000;
	Ps.Pitch.counter= 0;

    Ps.Yaw.Bise=0;
    Ps.Yaw.Integral_bias=0;           //��̨PID�б��ʼ��
    Ps.Yaw.Kp=0;                        pid_list[3]=&Ps.Yaw.Kp;
    Ps.Yaw.Ki=0;                        pid_list[4]=&Ps.Yaw.Ki;
    Ps.Yaw.Kd=0;                        pid_list[5]=&Ps.Yaw.Kd;
    Ps.Yaw.Last_Bise=0;
    Ps.Yaw.pid_out=0;
    Ps.Yaw.real=0;
    Ps.Yaw.Real_Angle=0;
    Ps.Yaw.RealCurrent=0;
    Ps.Yaw.target=4266;
	  Ps.Yaw.limit=&LtYaw;
    Ps.Yaw.limit->max=3000;
	Ps.Yaw.limit->min=-3000;
	Ps.Yaw.counter= 0;
		
		Ps.P=0;
		Ps.Y=0;
		Ps.motor_id=0x1FF;

/* ----------------- ��̨��Ա������ʼ�� -------------------- */
    Ps.PIDConnector=PIDConnector;

}
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  ��̨���ݽ���
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void PTZ_Data_Analysis(motor* RM6623,uint8_t *Data)
{
	static int16_t last_real_angle,Real_Angle;
    Real_Angle=((int16_t)(Data[0]<<8)|Data[1]);
	RM6623->RealCurrent=((int16_t)(Data[4]<<8)|Data[5]);
	/* ----------------- ����㴦�� -------------------- */
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
  * @brief  ��̨pid��ͬģʽ�ĳ�ʼ������
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
//void PTZ_PID_param(uint8_t way)
//{
//    switch(way)
//    {
//        case FIRST_POWER_UP:    //��һ���ϵ��ʼ��ģʽ
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
//   if((Ps.motor_id != L_Motor_Id) && (Ps.motor_id != H_Motor_Id))   		//id��Χ�Ƿ���1��8
//   {
//       hcan->pTxMsg->StdId = 0x00; //0~0x7ff���ó����id���ó���Ч���Է������������Ԥ�ϵ���
//       return 0x01;
//   }
//    hcan->pTxMsg->StdId = Ps.motor_id; //0~0x7ff
//    hcan->pTxMsg->IDE=CAN_ID_STD;//ѡ���׼id
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
//		 }while(!(hcan->Instance->TSR&(7<<26)));//�ȴ���������Ϊ��
//	 }
//   return 0x00;
//}
/* -------------------------------- begin -------------------------------- */
    /**
    * @brief  ��̨�õ�Ŀ��ֵ�Ļ�����
    * @param  
    * @retval 
    **/
/* -------------------------------- end -------------------------------- */
void PantiltzoomDbusDataConverter(void)
{
    Ps.Pitch.target=Ps.P+3700;
	Ps.Yaw.target=Ps.Y+4266;

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
