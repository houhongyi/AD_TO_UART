#ifndef _HPY_ADC_H
#define _HPY_ADC_H
#include "stm32f1xx_hal.h"

extern unsigned int AD_Valu[3];

void AD_Star();
void Encode(unsigned char code,
	GPIO_TypeDef* GPIOx0, uint16_t GPIO_Pin0,
	GPIO_TypeDef* GPIOx1, uint16_t GPIO_Pin1,
	GPIO_TypeDef* GPIOx2, uint16_t GPIO_Pin2);
#endif
