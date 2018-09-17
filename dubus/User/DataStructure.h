#ifndef __DATASTRUCTURE_H
#define __DATASTRUCTURE_H 

#include "stdio.h"
#include <stdint.h>
#include <stdlib.h>
extern unsigned char size_1;
#define MAXSIZE    8
typedef struct SqQueue{
	float data[MAXSIZE];
	unsigned char front;//队首指针
	unsigned char rear;//队尾指针
}SqQueue;


SqQueue GyinitQueue(void);
float isEmpty(SqQueue qu);
float enQueue(SqQueue *qu,float x,uint8_t size);
float deQueue(SqQueue *qu,float *y,uint8_t size);

void QueueOnit(void);
// int printQueue(SqQueue qu);
#endif	// __DATASTRUCTURE_H
