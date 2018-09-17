#include "niming.h"

Niming_Class  Debug;


/* -------------------------------- begin 1 -------------------------------- */
	/**
	* @brief  ���ڶ�����
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void usart2_send_char(uint8_t c,UART_HandleTypeDef* huart)
{
    while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TXE)==0);//�ȴ���һ�η������  
    huart->Instance->DR=c;   
}


/* -------------------------------- begin 2 -------------------------------- */
	/**
	* @brief  �����ṹ���ʼ��
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
	* @brief     ��λ���������ݸ�������λ��
    * @param / 1��uint8_t ������   0xA0~0xAF
    *        / 2��������������ָ�� ���24���ֽ�
    *        / 3������������Ч�������ݳ��� 
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void usart2_niming_report(uint8_t fun,uint8_t*data,uint8_t len,UART_HandleTypeDef* huart)
{
    uint8_t send_buf[32];
    uint8_t i;
    if(len>28)return;                                   //���28�ֽ�����
    send_buf[len+3]=0;                                  //У��������
    send_buf[0]=0X88;                                   //֡ͷ
    send_buf[1]=fun;                                    //������
    send_buf[2]=len;                                    //���ݳ���
    for(i=0;i<len;i++)send_buf[3+i]=data[i];            //��������
    for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];   //����У���
    for(i=0;i<len+4;i++)usart2_send_char(send_buf[i],huart);  //�������ݵ�����1
}

/* -------------------------------- begin 4 -------------------------------- */
   /**
    * @brief  ��������λ������PID����
    * @param / float Target_AngleĿ��Ƕ�
    *        / float Real_Angleʵ�ʽǶ�
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

    usart2_niming_report(0XA1,tbuf,8,huart);//�Զ���֡,0XA2
}
/* -------------------------------- begin 5 -------------------------------- */
   /**
    * @brief  ��������λ������PID����
    * @param / int16_t Target_AngleĿ��Ƕ�
    *        / int16_t Real_Angleʵ�ʽǶ�
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

    usart2_niming_report(0XA1,tbuf,4,huart);//�Զ���֡,0XA2
}



