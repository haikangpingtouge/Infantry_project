#include "DataStructure.h"
SqQueue Gysq;
SqQueue PTZsq;

unsigned char size_1=8;
void QueueOnit(void)
{
	Gysq = GyinitQueue();  //陀螺仪队列初始化
	PTZsq = GyinitQueue();  //云台外环pid计算出的值滤波
}

/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  创建队列结构体
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
SqQueue GyinitQueue(void)
{
	SqQueue *sq=(struct SqQueue*)malloc(sizeof(struct SqQueue));
	if(sq ==NULL)
		return (*sq);
	sq->rear=sq->front=0;
	return *sq;
}
/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  判断循环队列是否为空
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
float isEmpty(SqQueue qu)
{
	return (qu.front ==qu.rear?1:0);
}
/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  元素进循环队列
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
float enQueue(SqQueue *qu,float x,uint8_t size)
{
	if((qu->rear+1)%size ==qu->front){
		return 0;
	}
	qu->rear=(qu->rear+1)%size;
	qu->data[qu->rear]=x;
	return 1;
}
/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  元素出循环队列
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
float deQueue(SqQueue *qu,float *y,uint8_t size)
{
	if(qu->rear ==qu->front){
		return 0;
	}
	*y=qu->data[qu->front];
	qu->front=(qu->front+1)%size;
	return 1;
}
/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  打印循环队列
	* @param  
	* @retval 
	**/
/* -------------------------------- end -------------------------------- */
// int printQueue(SqQueue qu)
// {
// 	if(qu.rear ==qu.front){
// 		return 0;
// 	}
// 	while(qu.rear !=qu.front){
// 		qu.front=(qu.front+1)%MAXSIZE;
// 		printf("当前队列值=%d\n",qu.data[qu.front]);
// 	}
// 	return 1;
// }
