#ifndef __USER_H
#define __USER_H

#include "control.h"


 void User_config(void);
 void TIM7_Event(void);
 void User_usart1_IT(UART_HandleTypeDef *huart);
 void UserUsart6IT(UART_HandleTypeDef *huart);

 void BaseClassInit(Chassis_Struct* cs,Pantiltzoom_Struct* ps);
#endif


/* -------------------------------- begin -------------------------------- */
/**
  * @brief  ×¢ÊÍÄ£°å
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */



