#ifndef __USER_CAN_H
#define __USER_CAN_H
#include "Chassis.h"
#include "PanTiltZoom.h"


typedef struct
{
  uint8_t txdata[8];
	uint8_t rxdata[8];
}CAN_TxRx_message1;


typedef struct 
{
//	Chassis_Struct*   							CanCs;
//	Pantiltzoom_Struct*             canCPs;   
CAN_TxHeaderTypeDef*   Txm1;      //·¢ËÍ¾ä±ú
CAN_RxHeaderTypeDef*  Rxm1;      //½ÓÊÕ¾ä±ú
CAN_TxRx_message1*     Ctrm;	
}Can_Base_Class;
void User_CAN_Config(CAN_HandleTypeDef* hcan,Can_Base_Class* Cbc);
void CAN_Filter_Init(CAN_HandleTypeDef* hcan);
void CAN_Tx_Init(void);
void CAN_Rx_Init(void);
//typedef uint8_t(*Can_TX_Callback)(CAN_HandleTypeDef* hcan);
//uint8_t Can_Tx(Can_TX_Callback f,CAN_HandleTypeDef* hcan);



//void CAN_TX_RX_Config(CAN_HandleTypeDef* User_hcan);


#endif
