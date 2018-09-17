#include "niming.h"

Niming_Class  Debug;


/* -------------------------------- begin 1 -------------------------------- */
	/**
	* @brief  串口二发送
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void usart2_send_char(uint8_t c,UART_HandleTypeDef* huart)
{
    while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TXE)==0);//等待上一次发送完毕  
    huart->Instance->DR=c;   
}


/* -------------------------------- begin 2 -------------------------------- */
	/**
	* @brief  匿名结构体初始化
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void NimingClassInit(void)
{
	Debug.f=PID_Debugfloat;
	Debug.i16=PID_Debugint16_t;
}

/* -------------------------------- begin 3 -------------------------------- */
	/**
	* @brief     下位机发送数据给匿名上位机
    * @param / 1、uint8_t 功能字   0xA0~0xAF
    *        / 2、发送数据数组指针 最多24个字节
    *        / 3、发送数组有效区的数据长度 
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void usart2_niming_report(uint8_t fun,uint8_t*data,uint8_t len,UART_HandleTypeDef* huart)
{
    uint8_t send_buf[32];
    uint8_t i;
    if(len>28)return;                                   //最多28字节数据
    send_buf[len+3]=0;                                  //校验数置零
    send_buf[0]=0X88;                                   //帧头
    send_buf[1]=fun;                                    //功能字
    send_buf[2]=len;                                    //数据长度
    for(i=0;i<len;i++)send_buf[3+i]=data[i];            //复制数据
    for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];   //计算校验和
    for(i=0;i<len+4;i++)usart2_send_char(send_buf[i],huart);  //发送数据到串口1
}

/* -------------------------------- begin 4 -------------------------------- */
   /**
    * @brief  在匿名上位机调试PID函数
    * @param / float Target_Angle目标角度
    *        / float Real_Angle实际角度
    * @retval void
    **/
/* -------------------------------- end -------------------------------- */
void PID_Debugfloat(float Target,float Real,UART_HandleTypeDef* huart)
{
    uint8_t tbuf[8];
    unsigned char *p;
    p=(unsigned char *)&Target;
     tbuf[0]=(unsigned char)(*(p+3));
     tbuf[1]=(unsigned char)(*(p+2));
	 tbuf[2]=(unsigned char)(*(p+1));
	 tbuf[3]=(unsigned char)(*(p+0));

    p=(unsigned char *)&Real;
     tbuf[4]=(unsigned char)(*(p+3));
     tbuf[5]=(unsigned char)(*(p+2));
	 tbuf[6]=(unsigned char)(*(p+1));
	 tbuf[7]=(unsigned char)(*(p+0));
	  p=NULL;

    usart2_niming_report(0XA1,tbuf,8,huart);//自定义帧,0XA2
}
/* -------------------------------- begin 5 -------------------------------- */
   /**
    * @brief  在匿名上位机调试PID函数
    * @param / int16_t Target_Angle目标角度
    *        / int16_t Real_Angle实际角度
    * @retval void
    **/
/* -------------------------------- end -------------------------------- */
void PID_Debugint16_t(int16_t Target,int16_t Real,UART_HandleTypeDef* huart)
{
    uint8_t tbuf[4];
    unsigned char *p;
    p=(unsigned char *)&Target;
     tbuf[0]=(unsigned char)(*(p+1));
	 tbuf[1]=(unsigned char)(*(p+0));
	 
    p=(unsigned char *)&Real;
	tbuf[2]=(unsigned char)(*(p+1));
	tbuf[3]=(unsigned char)(*(p+0));
	  p=NULL;

    usart2_niming_report(0XA1,tbuf,4,huart);//自定义帧,0XA2
}



