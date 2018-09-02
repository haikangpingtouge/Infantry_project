#include "User_Can.h"
CAN_TxHeaderTypeDef   TxMsg1;      //���;��
CAN_RxHeaderTypeDef  RxMsg1;              //���վ��     
CAN_TxRx_message1 Cm;
CAN_FilterTypeDef _filter;
int dddd=0;
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  can����
  * @param  CAN_HandleTypeDef* hcan
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void User_CAN_Config(CAN_HandleTypeDef* hcan,Can_Base_Class* Cbc)   
{
	
   CAN_Filter_Init(hcan);  
   HAL_CAN_Start(hcan);
   HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);//�����ж�
	CAN_Tx_Init();
	CAN_Rx_Init();
	
	
	Cbc->Ctrm=&Cm;
	Cbc->Rxm1=&RxMsg1;
	Cbc->Txm1=&TxMsg1;
	
}


/* -------------------------------- begin -------------------------------- */
/**
  * @brief  can��������
  * @param  CAN_HandleTypeDef* hcan
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void CAN_Filter_Init(CAN_HandleTypeDef* hcan)
{
    
   _filter.FilterIdHigh         = 0x0000;				//Ҫ���˵�ID��λ 
   _filter.FilterIdLow          = 0x0000; //Ҫ���˵�ID��λ 
   _filter.FilterMaskIdHigh     = 0x0000;			//��������16λÿλ����ƥ��
   _filter.FilterMaskIdLow      = 0x0000;			//��������16λÿλ����ƥ��
   _filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;           //��������������FIFO 0
   _filter.FilterBank=0;
   _filter.FilterScale=CAN_FILTERSCALE_32BIT;
   _filter.FilterMode=CAN_FILTERMODE_IDMASK;
   _filter.FilterActivation = ENABLE;          //ʹ�ܹ�����
   _filter.SlaveStartFilterBank = 14;
	    
   	if(HAL_CAN_ConfigFilter(hcan,&_filter) != HAL_OK)			//ʹ��ɸѡ��
    {

    }

}


/* -------------------------------- begin -------------------------------- */
/**
  * @brief  can ���ͽṹ���ʼ��
  * @param  CanTxMsgTypeDef* TxMsg
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void CAN_Tx_Init()
{
    TxMsg1.StdId = 0x000; //����id
    TxMsg1.IDE = CAN_ID_STD; //ѡ���׼id
   TxMsg1.RTR = CAN_RTR_DATA; //0Ϊ����֡��1ΪԶ��֡
    TxMsg1.DLC = 8; //�������ݳ���Ϊ8���ֽ�
    Cm.txdata[0] = 0x00; //��������λ������
    Cm.txdata[1] = 0x00;
    Cm.txdata[2] = 0x00;
    Cm.txdata[3] = 0x00;
    Cm.txdata[4] = 0x00;
    Cm.txdata[5] = 0x00;
    Cm.txdata[6] = 0x00;
    Cm.txdata[7] = 0x00;
}


/* -------------------------------- begin -------------------------------- */
/**
  * @brief  can ���ܽṹ���ʼ��
  * @param  CanRxMsgTypeDef* RxMsg
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void CAN_Rx_Init(void)
{
    RxMsg1.StdId=0x00;
    RxMsg1.ExtId=0x00;
    RxMsg1.DLC=0x00;
    Cm.rxdata[0] = 0x00; //��������λ������
    Cm.rxdata[1] = 0x00;
    Cm.rxdata[2] = 0x00;
    Cm.rxdata[3] = 0x00;
    Cm.rxdata[4] = 0x00;
    Cm.rxdata[5] = 0x00;
    Cm.rxdata[6] = 0x00;
    Cm.rxdata[7] = 0x00;
}

