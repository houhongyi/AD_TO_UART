#include "HPY_adc.h"
#include "adc.h"

unsigned int AD_Valu[3]={0};

void AD_Star()
{
		HAL_ADC_Start_DMA(&hadc1,AD_Valu,3);
}

//对3个IO进行编码
void Encode(unsigned char code,
	GPIO_TypeDef* GPIOx0, uint16_t GPIO_Pin0,
	GPIO_TypeDef* GPIOx1, uint16_t GPIO_Pin1,
	GPIO_TypeDef* GPIOx2, uint16_t GPIO_Pin2)
{
	HAL_GPIO_WritePin(GPIOx0,GPIO_Pin0,code&0x01);
	HAL_GPIO_WritePin(GPIOx1,GPIO_Pin1,(code>>1)&0x01);
	HAL_GPIO_WritePin(GPIOx2,GPIO_Pin2,(code>>2)&0x01);
}

