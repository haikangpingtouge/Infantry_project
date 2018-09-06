/* -------------------------------- the architecture communicates begin -------------------------------- */
  /**
    *1、总的数据的初始化
    *2、遥控数据接收
    *3、can的数据接收
    *4、用户所有控制中断处理函数
    *5、用户总中断函数        
  **/
/* --------------------------------  the architecture communicates end -------------------------------- */
#include "User.h"

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern Chassis_Struct  Cs;  //底盘对象
extern Pantiltzoom_Struct Ps;	//云台对象

BASE _base;       				//基类对象
extern Can_Base_Class Cbcla;
uint32_t temp;

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  用户总使能函数  放在main函数初始化
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void User_config(void)
{
	/* ----------------- 用户初始化 -------------------- */
   BaseClassInit(&Cs,&Ps); //基类初始化
	User_CAN_Config(&hcan1,&Cbcla); 
	 Chassis_init();         //底盘数据初始化
	 PanTiltZoom_Init();       //云台数据初始化 
	 DBUS_Init();            //遥控参数初始化     
   TimeCounterInit();        //测试时间结构体初始化
	Gy955ClassDataInit();       //陀螺仪

   /* ----------------- 中断开启 -------------------- */
	HAL_TIM_Base_Start_IT(&htim7);
	
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE );//串口1空闲中断使能
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE );//串口3空闲中断使能
	
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
	
}

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  遥控数据接收中断 
  * @param  串口1结构体指针
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void User_usart1_IT(UART_HandleTypeDef *huart)
{
  __HAL_UART_CLEAR_IDLEFLAG(huart);
	if(huart->hdmarx->Instance->NDTR==DBUSBackLength)//接收18字节正确
	{
		  DBUS_DataDecoding();        //解码
	}
	HAL_UART_Receive_DMA(huart,DBUSBuffer,DBUSLength + DBUSBackLength);//设置接收地址和数据长度
}

/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  陀螺仪数据接收中断
	* @param  串口3结构体指针
	* @retval void
	**/
/* -------------------------------- end -------------------------------- */
void UserUsart6IT(UART_HandleTypeDef *huart)
{

	__HAL_UART_CLEAR_IDLEFLAG(huart);
	if(huart->hdmarx->Instance->NDTR== GY955BACKLEN)//接收18字节正确
	{
		 AnalysisGyro();
	}
	HAL_UART_Receive_DMA(huart,GB955Buffer,GY955BACKLEN+GY955LEN);//设置接收地址和数据长度
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
// 
// }

/* -------------------------------- begin -------------------------------- */
 /**
   * @brief  重定义can接受回调函数  中断函数里面有这个回调函数 can数据解析    
   * @param CAN_HandleTypeDef* _hcan
   * @retval void
  **/
/* -------------------------------- end -------------------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance==CAN1)
  {
     HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,Cbcla.Rxm1,Cbcla.Ctrm->rxdata);
		 Parsing_Motor_datas(&_base,&Cbcla);    //can数据解析
     
  }
  
}

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  TIM7用户中断   ##放在中断函数中
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void TIM7_Event(void)
{
//	Tc.aa=HAL_GetTick();
	switch(DBUS_ReceiveData.switch_left)
	{
		
		
		case 1:	
		 DUBS_Data_RX(&_base,&DBUS_ReceiveData);//遥控等数据得取
// 				Ps.PIDConnector(YAW,-3,0,0);
//	Ps.PIDConnector(PITCH,-3,-0.1,0);
    PanTiltZoom_Control(&Ps,&hcan1);//云台
//		Cs.PIDConnector(LB,0.5,0,0);
//		Cs.PIDConnector(LF,0.5,0,0);
//		Cs.PIDConnector(RB,0.5,0,0);
//		Cs.PIDConnector(RF,0.5,0,0);		
//   Chassis_Control(&Cs,&hcan1);//底盘
//		Tc.bb=HAL_GetTick()-Tc.aa;
//                              //拨弹
                            //摩擦轮
		
			break;
		case 2:
			Ps.PIDConnector(YAW,0,0,0);
		Ps.PIDConnector(PITCH,0,0,0);
		Cs.PIDConnector(LB,0,0,0);
		Cs.PIDConnector(LF,0,0,0);
		Cs.PIDConnector(RB,0,0,0);
		Cs.PIDConnector(RF,0,0,0);
		CAN_TRANSMIT(&hcan1,0x200,0,0,0,0);
		CAN_TRANSMIT(&hcan1,0x1ff,0,0,0,0);
		
			break;
	}

	
}



/* -------------------------------- begin -------------------------------- */
/**
  * @brief  中断回调函数
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim7)
    {
        TIM7_Event();					//控制函数
    }
    // else if(htim == &htim6)
    // {

    //     TIM6_Event();					//状态更新函数
    // }
}

/* -------------------------------- begin -------------------------------- */
  /**
  * @brief  基类初始化
  * @param  Chassis_Struct* cs底盘结构体指针 Pantiltzoom_Struct* ps云台结构体指针
  * @retval void
  **/
/* -------------------------------- end -------------------------------- */
void BaseClassInit(Chassis_Struct* cs,Pantiltzoom_Struct* ps)
{
  _base.ConCs=cs;
  _base.ConPs=ps;
}
