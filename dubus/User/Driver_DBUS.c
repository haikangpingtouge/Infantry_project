
#define  __DBUS_GLOBALS

#include "Driver_DBUS.h"
//#include "stm32f4xx_hal_uart.h"

// #define ClearShackTick      50      //消抖时长

Time_Counter Tc;
AccelerationStructFlag  Accesf;  //标志位对象
/* -------------------------------- begin 1 -------------------------------- */
/**
  * @brief  DBUS初始化
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void DBUS_Init(void)
{
		DBUSConnectStatus = Lost;
		DBUS_ReceiveData.ch1 = 0;
		DBUS_ReceiveData.ch2 = 0;
		DBUS_ReceiveData.ch3 = 0;
		DBUS_ReceiveData.ch4 = 0;

		DBUS_ReceiveData.mouse.x=0;
		DBUS_ReceiveData.mouse.y=0;
		DBUS_ReceiveData.mouse.z=0;
	
		DBUS_ReceiveData.mouse.press_left=0;
		DBUS_ReceiveData.mouse.press_right=0;
        
        DBUS_ReceiveData.mouse.jumppress_left=0;
		DBUS_ReceiveData.mouse.jumppress_right=0;
		

		DBUS_ReceiveData.keyBoard.key_code = 0;
		DBUS_ReceiveData.keyBoard.jumpkey_code = 0;

		DBUS_ReceiveData.counter=0;

		DBUS_ReceiveData.Asf=&Accesf;
	
}

/* -------------------------------- begin 2 -------------------------------- */
/**
  * @brief  DBUS数据解析
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void DBUS_DataDecoding(void)
{
		LASTDBUS_ReceiveData = DBUS_ReceiveData;
		
		DBUS_ReceiveData.ch1 = (DBUSBuffer[0] | DBUSBuffer[1]<<8) & 0x07FF;
		DBUS_ReceiveData.ch1 -= 1024;
		DBUS_ReceiveData.ch2 = (DBUSBuffer[1]>>3 | DBUSBuffer[2]<<5 ) & 0x07FF;
		DBUS_ReceiveData.ch2 -= 1024;
		DBUS_ReceiveData.ch3 = (DBUSBuffer[2]>>6 | DBUSBuffer[3]<<2 | DBUSBuffer[4]<<10) & 0x07FF;
		DBUS_ReceiveData.ch3 -= 1024;
		DBUS_ReceiveData.ch4 = (DBUSBuffer[4]>>1 | DBUSBuffer[5]<<7) & 0x07FF;		
		DBUS_ReceiveData.ch4 -= 1024;
		
		DBUS_ReceiveData.switch_left = ( (DBUSBuffer[5] >> 4)& 0x000C ) >> 2;
		DBUS_ReceiveData.switch_right =  (DBUSBuffer[5] >> 4)& 0x0003 ;
		
		DBUS_ReceiveData.mouse.x = DBUSBuffer[6] | (DBUSBuffer[7] << 8);	//x axis
		DBUS_ReceiveData.mouse.y = DBUSBuffer[8] | (DBUSBuffer[9] << 8);
		DBUS_ReceiveData.mouse.z = DBUSBuffer[10]| (DBUSBuffer[11] << 8);
		
		DBUS_ReceiveData.mouse.press_left 	= DBUSBuffer[12];	// is pressed?
		DBUS_ReceiveData.mouse.press_right 	= DBUSBuffer[13];
		
		DBUS_ReceiveData.keyBoard.key_code 	= DBUSBuffer[14] | DBUSBuffer[15] << 8; //key borad code
}


void TimeCounterInit(void)
{
	Tc.aa=0;
	Tc.bb=0;
}

/* -------------------------------- begin 3 -------------------------------- */
	/**
	* @brief  按键按下，换算成加速度
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
// uint8_t DBUS_CheckPush(uint8_t mode)
// {
//switch (mode) 
//{
//	case XAXIS:
//	if(DBUS_ReceiveData.keyBoard.jumpkey_code&(KEY_W||KEY_S))
//	{
//		switch (DBUSCheck()) 
//		{
//			case KEY_W:
//			Accesf.a++;
//			Accesf.a=Accesf.a > 50 ? 50 :Accesf.a;
//			Accesf.flagw = 1;                                      
//			break;
//			case KEY_S:
//			Accesf.a--;
//			Accesf.a= Accesf.a < -50 ? -50 :-Accesf.a;
//			Accesf.flage = 1;
//		default:
//			break;
//		}
//	}
//	else
//	{
//		Accesf.flagw = 0; 
//		Accesf.flags = 0; 
//	}

//	  return Accesf.a;


//	switch (DBUSCheck()) 
//	{
//		case KEY_W:
//		Accesf.a++;
//		Accesf.a=Accesf.a > 50 ? 50 :Accesf.a;
//		Accesf.flagw = 1;
//		   //解决如何判断按键松开                                       
//		break;
//		case KEY_S:
//		Accesf.a--;
//		Accesf.a= Accesf.a < -50 ? -50 :-Accesf.a;
//	default:
//		break;
//	}

//	  if( Accesf.flagw==1)
//	  Accesf.flagw=0;

//	  if(Accesf.flags==1)
//	  Accesf.flags=0;

//	  return Accesf.a;

//default:
//	break;
//}

//		
//}











//	//  if(DBUS_ReceiveData.keyBoard.key_code & Key==KEY_W)
//	//  {
//	// 	Accesf.a++;
//	// 	// 加减速时间段为半秒   中断十毫米 
//	// 	 return Accesf.a > 50 ? 50 :Accesf.a ;
//	//  }
//	//  else
//	//  {
//	// 	Accesf.a--;
//	// 	// 加减速时间段为半秒   中断十毫米
//	// 	return Accesf.a < 0 ? 0 :Accesf.a ;
//	//  }
// }
// 
///* -------------------------------- begin 4 -------------------------------- */
//	/**
//	* @brief  判断键盘是否按下
//	* @param  
//	* @retval 
//	**/
///* -------------------------------- end -------------------------------- */
// uint8_t DBUS_CheckJumpKey(uint16_t Key)
// {
//	 if(DBUS_ReceiveData.keyBoard.jumpkey_code & Key)
//	 {
//		 return 1;
//	 }
//	 else
//	 {
//		 return 0;
//	 }
// }
// 
// 
///* -------------------------------- begin 5 -------------------------------- */
//	/**
//	* @brief  判断鼠标是否按下
//	* @param  
//	* @retval 
//	**/
///* -------------------------------- end -------------------------------- */
// uint8_t DBUS_CheckJumpMouse(uint8_t Key)
// {
//	 if(Key)
//	 {
//		 return DBUS_ReceiveData.mouse.press_left;
//	 }
//	 else
//	 {
//		 return DBUS_ReceiveData.mouse.press_right;
//	 }
// }

// /* -------------------------------- begin 6 -------------------------------- */
//	 /**
//	 * @brief  检查按下什么键
//	 * @param  
//	 * @retval 
//	 **/
// /* -------------------------------- end -------------------------------- */
// 
//uint8_t DBUSCheck()
//{
//	if((DBUS_ReceiveData.keyBoard.jumpkey_code & KEY_W)==KEY_W)
//	return KEY_W;
//	else if((DBUS_ReceiveData.keyBoard.jumpkey_code & KEY_S)==KEY_S)
//	return KEY_S;


//	// else if((DBUS_ReceiveData.keyBoard.jumpkey_code & KEY_A)==KEY_A)
//	// return KEY_A;
//	// else if((DBUS_ReceiveData.keyBoard.jumpkey_code & KEY_D)==KEY_D)
//	// return KEY_D;
//	// else if((DBUS_ReceiveData.keyBoard.jumpkey_code & KEY_Q)==KEY_Q)
//	// return KEY_Q;
//	// else if((DBUS_ReceiveData.keyBoard.jumpkey_code & KEY_E)==KEY_E)
//	// return KEY_E;
//	// else if((DBUS_ReceiveData.keyBoard.jumpkey_code & KEY_CTRL)==KEY_CTRL)
//	// return KEY_CTRL;
//	// else if((DBUS_ReceiveData.keyBoard.jumpkey_code & KEY_SHIFT)==KEY_SHIFT)
//	// return KEY_SHIFT;
//}
