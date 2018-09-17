#include "DataStructure.h"
SqQueue Gysq;
SqQueue PTZsq;

unsigned char size_1=8;
void QueueOnit(void)
{
	Gysq = GyinitQueue();  //�����Ƕ��г�ʼ��
	PTZsq = GyinitQueue();  //��̨�⻷pid�������ֵ�˲�
}

/* -------------------------------- begin -------------------------------- */
	/**
	* @brief  �������нṹ��
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
	* @brief  �ж�ѭ�������Ƿ�Ϊ��
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
	* @brief  Ԫ�ؽ�ѭ������
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
	* @brief  Ԫ�س�ѭ������
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
	* @brief  ��ӡѭ������
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
// 		printf("��ǰ����ֵ=%d\n",qu.data[qu.front]);
// 	}
// 	return 1;
// }
