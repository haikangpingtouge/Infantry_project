#ifndef __FILTER_H
#define __FILTER_H 
#include "base_class.h"
#define SIZE 20//�����С
#define N 12//����ƽ���˲�����ƽ��ֵʱ��ȡ�ĵ���

void IIRFilter1(float *in,float *out);

float low_filter(float in);
float SmoothFilter(float data,SqQueue* Gy,uint8_t size);

#endif	// __FILTER_H
