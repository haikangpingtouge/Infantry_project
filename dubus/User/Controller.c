#include "Controller.h"
extern UART_HandleTypeDef huart2;
/* -------------------------------- begin -------------------------------- */
/**
  * @brief  PID控制器
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
		motor->Integral_bias=(motor->Integral_bias > (1000))?1000:motor->Integral_bias;			//限制积分
		motor->Integral_bias=(motor->Integral_bias<(-1000))?(-1000):motor->Integral_bias;


		motor->pid_out = motor->Kp * motor->Bise  + motor->Ki * motor->Integral_bias + motor->Kd * (motor->Bise - motor->Last_Bise);
		
		/* ----------------- 减缓响应速度 -------------------- */
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
  * @brief  限幅
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
  * @brief  模糊控制器
  * @param  void
  * @retval void
 **/
/* -------------------------------- end -------------------------------- */
// /* -------------------------------- begin -------------------------------- */
//   /**
//   * @brief  PID参数修改接口
//   * @param  
//   * @retval 
//   **/
// /* -------------------------------- end -------------------------------- */
// void TheInterfacesParameters()
// {

// }
/* -------------------------------- begin -------------------------------- */
  /**
  * @brief  此函数用于返回一个数的绝对值
  * @param  要取绝对值的参数a
  * @retval 返回入口参数a的绝对值
  **/
/* -------------------------------- end -------------------------------- */
float myabs(float a)
{

    return a<0?(-a):a;

}



/* ####################################### end ######################################### */   
void usart2_send_char(uint8_t c,UART_HandleTypeDef* huart)
{
    while(__HAL_UART_GET_FLAG(huart,UART_FLAG_TXE)==0);//等待上一次发送完毕  
    USART2->DR=c;   
}
/* ####################################### begin ####################################### */

/* ####################################### begin ####################################### */
/**
 * @brief  下位机发送数据给匿名上位机
 * @param / 1、uint8_t 功能字   0xA0~0xAF
 *        / 2、发送数据数组指针 最多24个字节
 *        / 3、发送数组有效区的数据长度
 * @retval void
 **/
/* ####################################### end ######################################### */
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
/* ####################################### begin ####################################### */
/**
 * @brief  在匿名上位机调试PID函数
 * @param / 1、uint16_t Target_Angle目标角度
 *        / 2、uint16_t Real_Angle实际角度
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

    usart2_niming_report(0XA1,tbuf,4,huart);//自定义帧,0XA2
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
