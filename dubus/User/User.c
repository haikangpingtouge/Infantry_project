/* -------------------------------- the architecture communicates begin -------------------------------- */
  /**
    *1���ܵ����ݵĳ�ʼ��
    *2��ң�����ݽ���
    *3��can�����ݽ���
    *4���û����п����жϴ�����
    *5���û����жϺ���        
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
extern Chassis_Struct  Cs;  //���̶���
extern Pantiltzoom_Struct Ps;	//��̨����

BASE _base;       				//�������
extern Can_Base_Class Cbcla;
uint32_t temp;

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  �û���ʹ�ܺ���  ����main������ʼ��
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void User_config(void)
{
	/* ----------------- �û���ʼ�� -------------------- */
   BaseClassInit(&Cs,&Ps); //�����ʼ��
	User_CAN_Config(&hcan1,&Cbcla); 
	 Chassis_init();         //�������ݳ�ʼ��
	 PanTiltZoom_Init();       //��̨���ݳ�ʼ�� 
	 DBUS_Init();            //ң�ز�����ʼ��     
   TimeCounterInit();        //����ʱ��ṹ���ʼ��
	Gy955ClassDataInit();       //������

   /* ----------------- �жϿ��� -------------------- */
	HAL_TIM_Base_Start_IT(&htim7);
	
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE );//����1�����ж�ʹ��
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE );//����3�����ж�ʹ��
	
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
	
}

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  ң�����ݽ����ж� 
  * @param  ����1�ṹ��ָ��
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void User_usart1_IT(UART_HandleTypeDef *huart)
{
  __HAL_UART_CLEAR_IDLEFLAG(huart);
	if(huart->hdmarx->Instance->NDTR==DBUSBackLength)//����18�ֽ���ȷ
	{
		  DBUS_DataDecoding();        //����
	}
	HAL_UART_Receive_DMA(huart,DBUSBuffer,DBUSLength + DBUSBackLength);//���ý��յ�ַ�����ݳ���
}

/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  ���������ݽ����ж�
	* @param  ����3�ṹ��ָ��
	* @retval void
	**/
/* -------------------------------- end -------------------------------- */
void UserUsart6IT(UART_HandleTypeDef *huart)
{

	__HAL_UART_CLEAR_IDLEFLAG(huart);
	if(huart->hdmarx->Instance->NDTR== GY955BACKLEN)//����18�ֽ���ȷ
	{
		 AnalysisGyro();
	}
	HAL_UART_Receive_DMA(huart,GB955Buffer,GY955BACKLEN+GY955LEN);//���ý��յ�ַ�����ݳ���
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
// 
// }

/* -------------------------------- begin -------------------------------- */
 /**
   * @brief  �ض���can���ܻص�����  �жϺ�������������ص����� can���ݽ���    
   * @param CAN_HandleTypeDef* _hcan
   * @retval void
  **/
/* -------------------------------- end -------------------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if(hcan->Instance==CAN1)
  {
     HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,Cbcla.Rxm1,Cbcla.Ctrm->rxdata);
		 Parsing_Motor_datas(&_base,&Cbcla);    //can���ݽ���
     
  }
  
}

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  TIM7�û��ж�   ##�����жϺ�����
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
		 DUBS_Data_RX(&_base,&DBUS_ReceiveData);//ң�ص����ݵ�ȡ
// 				Ps.PIDConnector(YAW,-3,0,0);
//	Ps.PIDConnector(PITCH,-3,-0.1,0);
    PanTiltZoom_Control(&Ps,&hcan1);//��̨
//		Cs.PIDConnector(LB,0.5,0,0);
//		Cs.PIDConnector(LF,0.5,0,0);
//		Cs.PIDConnector(RB,0.5,0,0);
//		Cs.PIDConnector(RF,0.5,0,0);		
//   Chassis_Control(&Cs,&hcan1);//����
//		Tc.bb=HAL_GetTick()-Tc.aa;
//                              //����
                            //Ħ����
		
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
  * @brief  �жϻص�����
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim7)
    {
        TIM7_Event();					//���ƺ���
    }
    // else if(htim == &htim6)
    // {

    //     TIM6_Event();					//״̬���º���
    // }
}

/* -------------------------------- begin -------------------------------- */
  /**
  * @brief  �����ʼ��
  * @param  Chassis_Struct* cs���̽ṹ��ָ�� Pantiltzoom_Struct* ps��̨�ṹ��ָ��
  * @retval void
  **/
/* -------------------------------- end -------------------------------- */
void BaseClassInit(Chassis_Struct* cs,Pantiltzoom_Struct* ps)
{
  _base.ConCs=cs;
  _base.ConPs=ps;
}
