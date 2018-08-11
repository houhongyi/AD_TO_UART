#ifndef __SystemState_H
#define __SystemState_H
#include "stm32f1xx_hal.h"


//HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
typedef enum
{
	SimUart_Ready=0,
	SimUart_Sending
}SimUartState_Def;

typedef struct
{
	SimUartState_Def State;
	char Bit_n;
	unsigned char data;
	
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
	TIM_HandleTypeDef* htim;

}SystemSimUART_Def;

typedef struct
{
	float SystemTime_ms;
	TIM_HandleTypeDef* htim;
	SystemSimUART_Def SimUart;
}SystemState_Def;

extern SystemState_Def SystemState;

void SystemStateInite(void);//��ʼ��ϵͳ�ṹ��
float GetSystemTime(void);//���ص�ǰϵͳ����ʱ�� ��λms
void HPY_Delay_us(int us);//��ʱ us ������

void Sent_One_Data(unsigned char* ch);
void Sent_String(unsigned char* ch);//�����ַ���
void Sent_Data(unsigned char* ch,int len);//�����ֽ�

#endif
