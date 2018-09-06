#include "Controller.h"
extern UART_HandleTypeDef huart2;
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  PID������
  * @param  motor *motor 
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
_PID _PID_Kind;
void PID_control(motor *motor,uint8_t PID)
{

//float bise=0;

    switch(PID)
    {
      case Location_PID:
	 motor->Bise = motor->target - motor->Real_Angle;

//      motor->Bise=myabs(bise);
//      bise=8191-motor->Bise;
//      if(motor->Bise > bise)
//      {
//        motor->Bise = bise;
//      }
//      if(motor->Real_Angle<=4096)
//      {
//        if(motor->Real_Angle > motor->target || 4096 < ( motor->target - motor->Real_Angle))
//                 motor->Bise = -motor->Bise;
//      }
//      else
//      {
//        if(motor->Real_Angle >  motor->target && 4096 > ( motor->Real_Angle - motor->target))
//         motor->Bise = -motor->Bise;

//      }


	
			


		motor->Integral_bias += motor->Bise;
		motor->Integral_bias=(motor->Integral_bias > (1000))?1000:motor->Integral_bias;			//���ƻ���
		motor->Integral_bias=(motor->Integral_bias<(-1000))?(-1000):motor->Integral_bias;


		motor->pid_out = motor->Kp * motor->Bise  + motor->Ki * motor->Integral_bias + motor->Kd * (motor->Bise - motor->Last_Bise);
		
		/* ----------------- ������Ӧ�ٶ� -------------------- */
		if(motor->Bise>500||motor->Bise<-500)
		{
			motor->pid_out= (int16_t)motor->pid_out/5;
		}
        motor->Last_Bise = motor->Bise;
      break;
      case Incremental_PID:
        motor->Bise = motor->target - motor->real ;
		if(myabs(motor->Bise)<myabs(motor->Last_Bise))    motor->Last_Bise=motor->Bise;
		if(myabs(motor->Last_Bise)<myabs(motor->before_last_bias))  motor->before_last_bias=motor->Last_Bise;

			
     motor->pid_out += motor->Kp * (motor->Bise - motor->Last_Bise) + motor->Ki * motor->Bise+motor->Kd*(motor->Bise-2*motor->Last_Bise+motor->before_last_bias);
			motor->before_last_bias = motor->Last_Bise;
        motor->Last_Bise = motor->Bise; 
				
      break;
    }
}

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  �޷�
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
void Limiter(int16_t *parameter,int16_t max,int16_t min)
{
  *parameter = *parameter > max ? max:*parameter;
  *parameter = *parameter < min ? min:*parameter;
}

/* -------------------------------- begin -------------------------------- */
/**
  * @brief  ģ��������
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
// /* -------------------------------- begin -------------------------------- */
//   /**
//   * @brief  PID�����޸Ľӿ�
//   * @param  
//   * @retval 
//   **/
// /* -------------------------------- end -------------------------------- */
// void TheInterfacesParameters()
// {

// }
/* -------------------------------- begin -------------------------------- */
  /**
  * @brief  �˺������ڷ���һ�����ľ���ֵ
  * @param  Ҫȡ����ֵ�Ĳ���a
  * @retval ������ڲ���a�ľ���ֵ
  **/
/* -------------------------------- end -------------------------------- */
float myabs(float a)
{

    return a<0?(-a):a;

}



/* ####################################### end ######################################### */   
void usart2_send_char(uint8_t c,UART_HandleTypeDef* huart)
{
    while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TXE)==0);//�ȴ���һ�η������  
    USART2->DR=c;   
}
/* ####################################### begin ####################################### */

/* ####################################### begin ####################################### */
/**
 * @brief  ��λ���������ݸ�������λ��
 * @param / 1��uint8_t ������   0xA0~0xAF
 *        / 2��������������ָ�� ���24���ֽ�
 *        / 3������������Ч�������ݳ���
 * @retval void
 **/
/* ####################################### end ######################################### */
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
/* ####################################### begin ####################################### */
/**
 * @brief  ��������λ������PID����
 * @param / 1��uint16_t Target_AngleĿ��Ƕ�
 *        / 2��uint16_t Real_Angleʵ�ʽǶ�
 * @retval void
 **/
/* ####################################### end ######################################### */
void PID_Debug(int16_t Target,int16_t Real,UART_HandleTypeDef* huart)
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


/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void OutPIDControl(int16_t t,int16_t r,Pantiltzoom_Struct* Ps)
{
	Ps->PitchSpeel.Bise = -t-r;
	Ps->PitchSpeel.Integral_bias += Ps->PitchSpeel.Bise;
	Ps->PitchSpeel.P_out=Ps->PitchSpeel.Kp*Ps->PitchSpeel.Bise;
	Ps->PitchSpeel.I_out=Ps->PitchSpeel.Ki*Ps->PitchSpeel.Integral_bias;
	Ps->PitchSpeel.D_out=Ps->PitchSpeel.Kd*(Ps->PitchSpeel.Bise-Ps->PitchSpeel.Last_Bise);
	

	Ps->PitchSpeel.pid_out = Ps->PitchSpeel.P_out+Ps->PitchSpeel.I_out+Ps->PitchSpeel.D_out;

	Limiter(&(Ps->PitchSpeel.pid_out),2000,-2000);

	Ps->PitchSpeel.Last_Bise=Ps->PitchSpeel.Bise;

}
