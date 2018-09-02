#include "User_Can.h"
CAN_TxHeaderTypeDef   TxMsg1;      //发送句柄
CAN_RxHeaderTypeDef  RxMsg1;              //接收句柄     
CAN_TxRx_message1 Cm;
CAN_FilterTypeDef _filter;
int dddd=0;
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  can配置
  * @param  CAN_HandleTypeDef* hcan
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void User_CAN_Config(CAN_HandleTypeDef* hcan,Can_Base_Class* Cbc)   
{
	
   CAN_Filter_Init(hcan);  
   HAL_CAN_Start(hcan);
   HAL_CAN_ActivateNotification(hcan,CAN_IT_RX_FIFO0_MSG_PENDING);//开启中断
	CAN_Tx_Init();
	CAN_Rx_Init();
	
	
	Cbc->Ctrm=&Cm;
	Cbc->Rxm1=&RxMsg1;
	Cbc->Txm1=&TxMsg1;
	
}


/* -------------------------------- begin -------------------------------- */
/**
  * @brief  can邮箱配置
  * @param  CAN_HandleTypeDef* hcan
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void CAN_Filter_Init(CAN_HandleTypeDef* hcan)
{
    
   _filter.FilterIdHigh         = 0x0000;				//要过滤的ID高位 
   _filter.FilterIdLow          = 0x0000; //要过滤的ID低位 
   _filter.FilterMaskIdHigh     = 0x0000;			//过滤器高16位每位必须匹配
   _filter.FilterMaskIdLow      = 0x0000;			//过滤器低16位每位必须匹配
   _filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;           //过滤器被关联到FIFO 0
   _filter.FilterBank=0;
   _filter.FilterScale=CAN_FILTERSCALE_32BIT;
   _filter.FilterMode=CAN_FILTERMODE_IDMASK;
   _filter.FilterActivation = ENABLE;          //使能过滤器
   _filter.SlaveStartFilterBank = 14;
	    
   	if(HAL_CAN_ConfigFilter(hcan,&_filter) != HAL_OK)			//使能筛选器
    {

    }

}


/* -------------------------------- begin -------------------------------- */
/**
  * @brief  can 发送结构体初始化
  * @param  CanTxMsgTypeDef* TxMsg
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void CAN_Tx_Init()
{
    TxMsg1.StdId = 0x000; //设置id
    TxMsg1.IDE = CAN_ID_STD; //选择标准id
   TxMsg1.RTR = CAN_RTR_DATA; //0为数据帧，1为远程帧
    TxMsg1.DLC = 8; //设置数据长度为8个字节
    Cm.txdata[0] = 0x00; //发送数据位都清零
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
  * @brief  can 接受结构体初始化
  * @param  CanRxMsgTypeDef* RxMsg
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void CAN_Rx_Init(void)
{
    RxMsg1.StdId=0x00;
    RxMsg1.ExtId=0x00;
    RxMsg1.DLC=0x00;
    Cm.rxdata[0] = 0x00; //接收数据位都清零
    Cm.rxdata[1] = 0x00;
    Cm.rxdata[2] = 0x00;
    Cm.rxdata[3] = 0x00;
    Cm.rxdata[4] = 0x00;
    Cm.rxdata[5] = 0x00;
    Cm.rxdata[6] = 0x00;
    Cm.rxdata[7] = 0x00;
}

