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

void SystemStateInite(void);//初始化系统结构体
float GetSystemTime(void);//返回当前系统运行时间 单位ms
void HPY_Delay_us(int us);//延时 us 阻塞用

void Sent_One_Data(unsigned char* ch);
void Sent_String(unsigned char* ch);//发送字符串
void Sent_Data(unsigned char* ch,int len);//发送字节

#endif
