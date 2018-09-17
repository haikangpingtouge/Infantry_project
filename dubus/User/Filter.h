#ifndef __FILTER_H
#define __FILTER_H 
#include "base_class.h"
#define SIZE 20//数组大小
#define N 12//滑动平均滤波计算平均值时所取的点数

void IIRFilter1(float *in,float *out);

float low_filter(float in);
float SmoothFilter(float data,SqQueue* Gy,uint8_t size);

#endif	// __FILTER_H
