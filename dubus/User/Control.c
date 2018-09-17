#include "Control.h"
extern UART_HandleTypeDef huart2;
extern SqQueue Gysq;
extern SqQueue PTZsq;
Can_Base_Class Cbcla;
uint8_t groal[2]={90,90};
int aff=0;
float lowbuffer;
 int16_t  aaaaaa;
uint8_t size_2=5;
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  ��̨����
  * @param  PTZ* This ��̨�ṹ��ָ��
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void PanTiltZoom_Control(Pantiltzoom_Struct* This,CAN_HandleTypeDef* hcan)
{
  //  if(FIRST_POWER_UP) 
  //  {
  //    PTZ_PID_param(FIRST_POWER_UP); 
  //  }
//	This->PIDConnector(PITCH,0,0,0);       //����˵��:1��������ͣ�2��Kp 3��Ki  4��Kd
 PantiltzoomDbusDataConverter();                    //��̨ң�����ݵ�ȡ
//	FRune(); //  ���ģʽ

//  Limiter(&This->Pitch.target,This->Pitch.limit->max,This->Pitch.limit->min);
//  Control_PTZ_PID(This);             //��̨PID����
//	Limiter(&This->Yaw.target,This->Yaw.limit->max,This->Yaw.limit->min);
  Control_PTZ_PID(This);             //��̨PID����
	CAN_TRANSMIT(hcan,0x1ff,This->Yaw.pid_out,This->Pitch.pid_out,0,0);
//  PanTiltZoomCanTX(hcan);
//	PID_Debug(This->Pitch.pid_out,This->Pitch.Real_Angle,&huart2);
//	PID_Debug(This->Yaw.pid_out,This->Yaw.Real_Angle,&huart2);
	

}

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  ���̿���
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void Chassis_Control(Chassis_Struct* This,CAN_HandleTypeDef* hcan_1)
{
	
//	This->PIDConnector(LF,1,2,3);

    MW_TheCalculationSpeed(This->Target_Vx, This->Target_Vy, This->Target_Wt, This);// �����˶�ģ�ͽ���,�ó�Ŀ��ֵ
    Control_Chassis_PI(This);//PID����
	
	CAN_TRANSMIT(hcan_1,0x200,This->chassis_LF.pid_out,This->chassis_LB.pid_out,This->chassis_RB.pid_out,This->chassis_RF.pid_out);
//	CAN_TRANSMIT(hcan_1,0x200,This->chassis_LF.pid_out,0,0,0);
//	PID_Debug(This->chassis_LF.target,This->chassis_LF.pid_out,&huart2);
//    Can_Tx(CHASSISDATA,hcan_1);  //can������̨��������

}


/* -------------------------------- begin -------------------------------- */
/**
  * @brief  ����pid
  * @param  Chassis_Struct* _Chassis ���̽ṹ��ָ��
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void Control_Chassis_PI(Chassis_Struct* _Chassis)
{
   PID_control(&(_Chassis->chassis_LB),Incremental_PID);  
   Limiter(&_Chassis->chassis_LB.pid_out,_Chassis->max,_Chassis->min);

   PID_control(&(_Chassis->chassis_LF),Incremental_PID);
   Limiter(&_Chassis->chassis_LF.pid_out,_Chassis->max,_Chassis->min);
//   PID_Debug(_Chassis->chassis_LF.real,_Chassis->chassis_LF.target,&huart2);

   PID_control(&(_Chassis->chassis_RB),Incremental_PID);
   Limiter(&_Chassis->chassis_RB.pid_out,_Chassis->max,_Chassis->min);

   PID_control(&(_Chassis->chassis_RF),Incremental_PID);
   Limiter(&_Chassis->chassis_RF.pid_out,_Chassis->max,_Chassis->min);
}

/* -------------------------------- begin -------------------------------- */
  /**
  * @brief  ��̨pid
  * @param  
  * @retval 
  **/
/* -------------------------------- end -------------------------------- */
void Control_PTZ_PID(Pantiltzoom_Struct* This)
{
	/* ----------------- �⻷Pitch  PID -------------------- */
//	  Limiter(&This->Pitch.target,This->Pitch.limit->max,This->Pitch.limit->min);
//    PID_control(&This->Pitch,Location_PID);
	
	/* ----------------- ר��Pitch  PID -------------------- */
		  ExpertPIDControl(&This->Pitch);
    
   Limiter(&This->Pitch.pid_out,This->Pitch.limit->max,This->Pitch.limit->min);
	aaaaaa = This->Pitch.pid_out;
   /* ----------------- �⻷Yawpid -------------------- */

//   PID_Debug(This->Pitch.target,This->Pitch.Real_Angle,&huart2);

//    PID_control(&This->Yaw,Location_PID);
//	ExpertPIDControl(&This->Yaw);
		/* ----------------- ר��Yaw  PID -------------------- */
//	ExpertPIDControl(&This->Yaw);
//	  Limiter(&This->Yaw.pid_out,This->Yaw.limit->max,This->Yaw.limit->min);
//	  PID_Debug(This->Yaw.target,This->Yaw.Real_Angle,&huart2);
//	  /* ----------------- �ڻ�Pitch pid -------------------- */

		//    lowbuffer=low_filter(This->Gyc->Gyr_y);             //��ͨ�˲�
		lowbuffer = SmoothFilter(This->Gyc->Gyr_y,&Gysq,size_1);//ƽ���˲�
//		Debug.i16(lowbuffer,This->Gyc->Gyr_y,&huart2);   //����


 
	  OutPIDControl(This->Pitch.pid_out,lowbuffer,This);
//	This->Pitch.pid_out = SmoothFilter(This->PitchSpeel.pid_out,&PTZsq,size_2);   //ƽ���˲�
This->Pitch.pid_out = low_filter(This->PitchSpeel.pid_out); //��ͨ�˲�
//	Debug.i16(aaaaaa,This->Pitch.pid_out,&huart2);
	
////	  /* ----------------- �ڻ�Yaw PID -------------------- */
//	OutPIDControl(This->Yaw.pid_out,This->Gyc->Gyr_z,This);
////	This->Pitch.pid_out = low_filter(This->PitchSpeel.pid_out);
//	This->Yaw.pid_out=This->PitchSpeel.pid_out;
//	
	
}


/* -------------------------------- begin -------------------------------- */
/**
  * @brief  ������ݽ���
  * @param  CanRxMsgTypeDef* RxMsg
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void Parsing_Motor_datas(BASE* This, Can_Base_Class* Cbc)
{
   volatile uint32_t temp;
   temp = Cbc->Rxm1->StdId; //�õ����ID
   switch (temp)
   {

       case 0x201: 
        Chassis_Data_Analysis(&(This->ConCs->chassis_LF  ),Cbc->Ctrm->rxdata);
        break;
       case 0x202:
        Chassis_Data_Analysis(&(This->ConCs->chassis_LB),Cbc->Ctrm->rxdata);
        break;
       case 0x203:
        Chassis_Data_Analysis(&(This->ConCs->chassis_RB),Cbc->Ctrm->rxdata);
        break;
       case 0x204:
        Chassis_Data_Analysis(&(This->ConCs->chassis_RF),Cbc->Ctrm->rxdata);
        break;
       case 0x205:
         PTZ_Data_Analysis(&(This->ConPs->Yaw),Cbc->Ctrm->rxdata);
        break;
       case 0x206:
				  PTZ_Data_Analysis(&(This->ConPs->Pitch),Cbc->Ctrm->rxdata);

        break;

		default:
		break;


    }

 }
 



 /* -------------------------------- begin -------------------------------- */
/**
  * @brief  ң��ģʽ���ݴ���
  * @param  BASE* This����ṹ��ָ��  DBUSDecoding_Type *dataң�ؽṹ��ָ��
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
float SpeedParam=12;//�����ٶȱ���
void DUBS_Data_RX(BASE* This,DBUSDecoding_Type *data)
{
//	static int16_t Y,P;
		//����X��Y�����ٶ�
//		This->ConCs->Target_Vx=data->ch2*SpeedParam;
//		This->ConCs->Target_Vy=data->ch1*SpeedParam;
//    This->ConPs->Y=-data->ch3;
//    This->ConPs->P=-data->ch4;
	/* -------------------------------- ���ӳ� -------------------------------- */
			if(data->ch4<20&&data->ch4>-20) //��ֹң�����ݲ��ܹ���
		data->ch4=0;
		if(data->ch3<20&&data->ch3>-20)
		data->ch3=0;
				if(data->ch1<20&&data->ch1>-20)
		data->ch1=0;
				This->ConCs->Target_Vx = (int16_t)data->ch4*SpeedParam;
		This->ConCs->Target_Vy = (int16_t)data->ch3*SpeedParam;
		This->ConCs->Target_Wt = (int16_t)data->ch1*SpeedParam;
		
		This->ConCs->Target_Vx = LimiterInt16(This->ConCs->Target_Vx,8000,-8000);
		This->ConCs->Target_Vy = LimiterInt16(This->ConCs->Target_Vy,8000,-8000);
		This->ConCs->Target_Wt = LimiterInt16(This->ConCs->Target_Wt,5000,-5000);
		This->ConPs->P += (int16_t)(-data->ch2)*0.01;
				This->ConPs->P =LimiterInt16(This->ConPs->P,2000,-700);
				
/* -------------------------------- �Ը�ģʽ -------------------------------- */
//		This->ConCs->Target_Vx=data->ch4*SpeedParam;
//		This->ConCs->Target_Vy=data->ch3*SpeedParam;
//    This->ConPs->Y=-data->ch1;
//    This->ConPs->P=-data->ch2;
//	
/* -------------------------------- ������ģʽ -------------------------------- */
//		if(data->ch4<20&&data->ch4>-20) //��ֹң�����ݲ��ܹ���
//		data->ch4=0;
//		if(data->ch3<20&&data->ch3>-20)
//		data->ch3=0;
//		
//		
//		if(data->ch2>50&&data->ch2<-50)
//		{
//			if(data->ch2>50)
//				data->ch2=50;
//			if(data->ch2<-50)
//				data->ch2=-50;
//				
//		}
//			
//				if(data->ch1>50&&data->ch1<-50)
//				{
//				if(data->ch1>50)
//				data->ch1=50;
//			if(data->ch1<-50)
//				data->ch1=-50;
//				}
////		if(data->ch3<20&&data->ch3>-20)
////		data->ch3=0;
////		if(data->ch4<20&&data->ch4>-20) //��ֹң�����ݲ��ܹ���
////		data->ch4=0;

//		This->ConCs->Target_Vx = (int16_t)data->ch4*SpeedParam;
//		This->ConCs->Target_Wt = (int16_t)data->ch3*SpeedParam;
//		
//		This->ConCs->Target_Vx = LimiterInt16(This->ConCs->Target_Vx,8000,-8000);
//		This->ConCs->Target_Wt = LimiterInt16(This->ConCs->Target_Wt,5000,-5000);

//    This->ConPs->Y += (int16_t)(-data->ch1)*0.02;
//	This->ConPs->P += (int16_t)(-data->ch2)*0.002;
//	
//	This->ConPs->Y = LimiterInt16(This->ConPs->Y,650,-650);
//	This->ConPs->P =LimiterInt16(This->ConPs->P,175,-175);
}

///* -------------------------------- begin -------------------------------- */
//    /**
//    * @brief  �Զ���can���ͺ���
//    * @param  Can_TX_Callback f �����������ͺ��� CAN_HandleTypeDef* hcan_1
//    * @retval 
//    **/
///* -------------------------------- end -------------------------------- */
//uint8_t Can_Tx(Can_TX_Callback f,CAN_HandleTypeDef* hcan)
//{
//    return f(hcan);
//}

 void CAN_TRANSMIT(CAN_HandleTypeDef* hcan,int STD_ID,int Letter0,int Letter1,int Letter2,int Letter3)
 {
//	 int sLetter3;
//	sLetter3=Letter0;
  Cbcla.Txm1->StdId=STD_ID;
  Cbcla.Txm1->IDE=CAN_ID_STD;
  Cbcla.Txm1->RTR=CAN_RTR_DATA;
	   Cbcla.Txm1->DLC=0x08;
	 
   Cbcla.Ctrm->txdata[0] = (Letter0>>8);
	Cbcla.Ctrm->txdata[1] = (Letter0);
	Cbcla.Ctrm->txdata[2] = (Letter1>>8) & 0xFF;
	Cbcla.Ctrm->txdata[3] = (Letter1) & 0xFF;
	Cbcla.Ctrm->txdata[4] = (Letter2>>8) & 0xFF;
	Cbcla.Ctrm->txdata[5] = (Letter2) & 0xFF;
	Cbcla.Ctrm->txdata[6] = (Letter3>>8) & 0xFF;
	Cbcla.Ctrm->txdata[7] = (Letter3) &0xFF;

   if(HAL_CAN_AddTxMessage(hcan,Cbcla.Txm1,Cbcla.Ctrm->txdata,(uint32_t*)CAN_TX_MAILBOX0)==HAL_OK)
  {
     
//		if((HAL_GetTick()-Tc.aa)>5)   Tc.bb= HAL_GetTick()-Tc.aa;
		
  }
}
 

/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void FRune(void)
{

	HAL_UART_Receive(&huart2,groal,2,100);
	RuneController(groal);
	aff=1;
	

}

void CHUANKOU(void)
{
	HAL_UART_Transmit(&huart2,groal,2,100);
	
}
