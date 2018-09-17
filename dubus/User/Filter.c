#include "Filter.h"
float pamer,lastBiseGy; 
float X=0.36;
float w_x[3],w_y[3],A[3],B[3],Gain;


float Sum=0, Sum1=0,Su=0;
/* -------------------------------- begin 1 -------------------------------- */
	/**
	* @brief  ÂË²¨
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
void IIRFilter1(float *in,float *out)
{
	*out = *out + pamer*(*in - *out);
}


/* -------------------------------- begin 2 -------------------------------- */
	/**
	* @brief  µÍÍ¨Êý×ÖÂË²¨
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
float low_filter(float in)
{

    float sample_value;
	sample_value=(1-X)*in+X*lastBiseGy;
	lastBiseGy=in;
    return(sample_value);
}


/* -------------------------------- begin 3 -------------------------------- */
	/**
	* @brief  »¬¶¯Æ½¾ùÂË²¨
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
float SmoothFilter(float data,SqQueue* Gy,uint8_t size)
{
	int j=0;
	enQueue(Gy,data,size);
	Sum1=0;
	
    for( j=0;j<size_1;j++)
    {
			Sum1 += Gy->data[j];
   }
		Sum = Sum1/size_1;
	 Gy->data[Gy->rear] = Sum;
	 deQueue(Gy,&Su,size);


   return Sum;
 }

// 
//         if(j<N/2)
//        {
//            for( k=0;k<N;k++)
//            {
//                Sum1+=Gy->data[j+k];
//            }
//            Gy->data[j]=Sum1/N;
//        }
//        else
//            if(j<size -N/2)
//            {
//                for(k=0;k<N/2;k++)
//                {
//                    Sum1+=(Gy->data[j+k]+Gy->data[j-k]);
//                }
//				Gy->data[j]=Sum1/N;
//            }
//            else
//            {
//                for(k=0;k<size-j;k++)
//                {
//                    Sum1+=Gy->data[j+k];
//                }
//                for(k=0;k<(N-size+j);k++)
//                {
//                    Sum1+=Gy->data[j-k];
//                }
//                Gy->data[j]=Sum1/N;
//            }

