#include "Chassis.h"
Chassis_Struct  Cs;  //底盘对象
static float *pid_list[12];

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  底盘电机数据解析
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void Chassis_Data_Analysis(motor* RM3508,uint8_t *Data)
{
    RM3508->Real_Angle=((int16_t)(Data[0]<<8)|Data[1]);
    RM3508->real=((int16_t)(Data[2]<<8)|Data[3]);
    RM3508->RealCurrent=((int16_t)(Data[4]<<8)|Data[5]);
}
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  麦轮速度解算
  * @param   Vx x轴速度,  Vy y轴速度,  Wt 角速度, *Speed 传入速度指针
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
int16_t Buffer[4],MaxSpeed;
float  Param;


void MW_TheCalculationSpeed(float Vx, float Vy, float Wt,Chassis_Struct* D)
{
    uint8_t index;
    Buffer[0] = Vx + Vy + Wt;
    Buffer[1] = Vx - Vy - Wt;
    Buffer[2] = Vx - Vy + Wt;
    Buffer[3] = Vx + Vy - Wt;

    for(index = 0, MaxSpeed = 0; index < 4; index++)
    {
        if((Buffer[index] > 0 ? Buffer[index] : -Buffer[index]) > MaxSpeed)
        {
            MaxSpeed = (Buffer[index] > 0 ? Buffer[index] : -Buffer[index]);//挑出最大的速度
        }
    }
    if(MaxWheelSpeed < MaxSpeed)
    {
        Param = (float)MaxWheelSpeed / MaxSpeed;
        D->chassis_LF.target = Buffer[0] * Param;
        D->chassis_LB.target = Buffer[1] * Param;
			  D->chassis_RF.target = -Buffer[2] * Param; 
        D->chassis_RB.target = -Buffer[3] * Param;
   
    }
    else
    {
        D->chassis_LF.target = Buffer[0];
        D->chassis_LB.target = Buffer[1];
			 D->chassis_RF.target = -Buffer[2];
        D->chassis_RB.target =-Buffer[3];
       
    }
}


/* -------------------------------- begin -------------------------------- */
/**
  * @brief  底盘电机类初始化
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void Chassis_init(void)
{
/* ----------------- 公有数据初始化 -------------------- */
    Cs.Target_Vx=0;  //遥控数据缓存
    Cs.Target_Vy=0;
    Cs.Target_Wt=0;	//遥控数据缓存
    Cs.max=1000;          // 限制值
    Cs.min=-1000;          // 限制值
    Cs.motor_id=0x200;


     Cs.chassis_LF.target=0;             //电机目标速度
     Cs.chassis_LF.real=0;               //电机实际速度
     Cs.chassis_LF.RealCurrent=0;            //实际电流
     Cs.chassis_LF.Real_Angle=0;             //;
     Cs.chassis_LF.pid_out=0;    
     Cs.chassis_LF.Kp=3.5;             pid_list[0]= &Cs.chassis_LF.Kp; //得到pid参数 
     Cs.chassis_LF.Ki=0.4;             pid_list[1]= &Cs.chassis_LF.Ki;  
     Cs.chassis_LF.Kd=0.05;                pid_list[2]= &Cs.chassis_LF.Kd;   
     Cs.chassis_LF.Bise=0;
     Cs.chassis_LF.Last_Bise=0; 
     Cs.chassis_LF.Integral_bias=0;
     Cs.chassis_LF.limit=NULL;
		 Cs.chassis_LF.before_last_bias=0;
		 Cs.chassis_LF.counter=0;

     Cs.chassis_RF.target=0;             //电机目标速度
     Cs.chassis_RF.real=0;               //电机实际速度
     Cs.chassis_RF.RealCurrent=0;            //实际电流
     Cs.chassis_RF.Real_Angle=0;             //;
     Cs.chassis_RF.pid_out=0;  
     Cs.chassis_RF.Kp=3.5;             pid_list[3]= &Cs.chassis_RF.Kp;   
     Cs.chassis_RF.Ki=0.4;             pid_list[4]= &Cs.chassis_RF.Ki;    
     Cs.chassis_RF.Kd=0.05;                pid_list[5]=&Cs.chassis_RF.Kd;
     Cs.chassis_RF.Bise=0;
     Cs.chassis_RF.Last_Bise=0; 
     Cs.chassis_RF.Integral_bias=0;
     Cs.chassis_RF.limit=NULL;     
 Cs.chassis_RF.before_last_bias=0;
 Cs.chassis_RF.counter=0;
 

     Cs.chassis_LB.target=0;             //电机目标速度
     Cs.chassis_LB.real=0;               //电机实际速度
     Cs.chassis_LB.RealCurrent=0;            //实际电流
     Cs.chassis_LB.Real_Angle=0;             //;
     Cs.chassis_LB.pid_out=0;  
     Cs.chassis_LB.Kp=3.5;                    pid_list[6]= &Cs.chassis_LB.Kp;   
     Cs.chassis_LB.Ki=0.4;                    pid_list[7]= &Cs.chassis_LB.Ki;    
     Cs.chassis_LB.Kd=0.05;                    pid_list[8]= &Cs.chassis_LB.Kd;     
     Cs.chassis_LB.Bise=0;
     Cs.chassis_LB.Last_Bise=0; 
     Cs.chassis_LB.Integral_bias=0;
     Cs.chassis_LB.limit=NULL;
 Cs.chassis_LB.before_last_bias=0;
 Cs.chassis_LB.counter=0;
 
     

     Cs.chassis_RB.target=0;             //电机目标速度
     Cs.chassis_RB.real=0;               //电机实际速度
     Cs.chassis_RB.RealCurrent=0;            //实际电流
     Cs.chassis_RB.Real_Angle=0;             //;
     Cs.chassis_RB.pid_out=0;  
     Cs.chassis_RB.Kp=3.5;             pid_list[9]=  &Cs.chassis_RB.Kp;
     Cs.chassis_RB.Ki=0.4;             pid_list[10]=  &Cs.chassis_RB.Ki;
     Cs.chassis_RB.Kd=0.05;                pid_list[11]= &Cs.chassis_RB.Kd;
     Cs.chassis_RB.Bise=0;
     Cs.chassis_RB.Last_Bise=0; 
     Cs.chassis_RB.Integral_bias=0;
     Cs.chassis_RB.limit=NULL;	 
		  Cs.chassis_RB.before_last_bias=0;
		  Cs.chassis_RB.counter=0;
/* ----------------- 成员函数初始化 -------------------- */
     Cs.PIDConnector=PIDConnector;
}

/* -------------------------------- begin -------------------------------- */
 /**
   * @brief  can的底盘数据发送  
   * @param 
   * @retval void
  **/
/* -------------------------------- end -------------------------------- */
//uint8_t ChassisCanTX(CAN_HandleTypeDef* hcan_1)
//{
//    if((Cs.motor_id != L_Motor_Id) && (Cs.motor_id != H_Motor_Id))   		//id范围是否在1到8
//    {
//        hcan_1->pTxMsg->StdId = 0x00; //0~0x7ff设置出错后id设置成无效，以防电机发生不可预料的事
//        return 0x01;
//    }
//     hcan_1->pTxMsg->StdId = Cs.motor_id; //0~0x7ff
//     hcan_1->pTxMsg->IDE=CAN_ID_STD;//选择标准id
//     hcan_1->pTxMsg->RTR=0x00;
//     hcan_1->pTxMsg->DLC=8;

//     hcan_1->pTxMsg->Data[0] = (Cs.chassis_LF.pid_out>> 8) & 0xFF;
//     hcan_1->pTxMsg->Data[1] = Cs.chassis_LF.pid_out & 0xFF;
//     hcan_1->pTxMsg->Data[2] = (Cs.chassis_LB.pid_out >> 8) & 0xFF;
//     hcan_1->pTxMsg->Data[3] = Cs.chassis_LB.pid_out & 0xFF;
//     hcan_1->pTxMsg->Data[4] = (Cs.chassis_RB.pid_out >> 8) & 0xFF;
//     hcan_1->pTxMsg->Data[5] = Cs.chassis_RB.pid_out & 0xFF;
//     hcan_1->pTxMsg->Data[6] = (Cs.chassis_RF.pid_out >> 8) & 0xFF;
//     hcan_1->pTxMsg->Data[7] = Cs.chassis_RB.pid_out & 0xFF;

//    HAL_CAN_Transmit(hcan_1, 1000);
//    return 0x00;
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
