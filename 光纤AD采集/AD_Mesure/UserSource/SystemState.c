#include "SystemState.h"
#include "tim.h"
SystemState_Def SystemState={0};

void SystemStateInite()//��ʼ��ϵͳ�ṹ��
{

	SystemState.htim=&htim3;
	SystemState.SimUart.htim=&htim1;
	
	SystemState.SimUart.GPIOx=GPIOA;
	SystemState.SimUart.GPIO_Pin=GPIO_PIN_13;
	
	HAL_TIM_Base_Start_IT(SystemState.htim);
	HAL_TIM_Base_Start_IT(SystemState.SimUart.htim);
}

float GetSystemTime()//���ص�ǰϵͳ����ʱ�� ��λms
{
		return SystemState.SystemTime_ms+(SystemState.htim->Instance->CNT/1000.0f);
}

void HPY_Delay_us(int us)//��ʱ us ������
{
	float time_Star=GetSystemTime()+(us/1000.0f);
	while(1)
	{
		if(time_Star<=GetSystemTime())break;
	}
}



//����һ���ַ�
void Sent_One_Data(unsigned char* ch)
{
		while(SystemState.SimUart.State==SimUart_Sending);
		SystemState.SimUart.Bit_n=(char)-1;
		SystemState.SimUart.data=*ch;
		SystemState.SimUart.State=SimUart_Sending;
		while(SystemState.SimUart.State==SimUart_Sending);
}

//�����ַ���
void Sent_String(unsigned char* ch)
{
		while(*ch!=0)
		{
			Sent_One_Data(ch);
			ch++;
		}
}
//�����ֽ�
void Sent_Data(unsigned char* ch,int len)
{
		while(len--)
		{
			Sent_One_Data(ch);
			ch++;
		}
}

char IsBit(unsigned char ch,unsigned char n)
{
		if(ch & (1<<n))
			return 1;
		else
			return 0;
}

void SimulateUARTInISR()
{
	if(SystemState.SimUart.State==SimUart_Sending)
	{
		switch(SystemState.SimUart.Bit_n)
		{
			case 0xff://���͵͵�ƽ ����λ
					HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,GPIO_PIN_RESET);
				break;
			case 0:HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,(GPIO_PinState)IsBit(SystemState.SimUart.data,SystemState.SimUart.Bit_n));
				break;
			case 1:HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,(GPIO_PinState)IsBit(SystemState.SimUart.data,SystemState.SimUart.Bit_n));
				break;
			case 2:HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,(GPIO_PinState)IsBit(SystemState.SimUart.data,SystemState.SimUart.Bit_n));
				break;
			case 3:HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,(GPIO_PinState)IsBit(SystemState.SimUart.data,SystemState.SimUart.Bit_n));
				break;
			case 4:HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,(GPIO_PinState)IsBit(SystemState.SimUart.data,SystemState.SimUart.Bit_n));
				break;
			case 5:HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,(GPIO_PinState)IsBit(SystemState.SimUart.data,SystemState.SimUart.Bit_n));
				break;
			case 6:HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,(GPIO_PinState)IsBit(SystemState.SimUart.data,SystemState.SimUart.Bit_n));
				break;
			case 7:HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,(GPIO_PinState)IsBit(SystemState.SimUart.data,SystemState.SimUart.Bit_n));
				break;
			case 8://���͸ߵ�ƽ ֹͣλ
				HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,GPIO_PIN_SET);
				SystemState.SimUart.State=SimUart_Ready;
				SystemState.SimUart.Bit_n=(char)-2;
				break;
		}
		SystemState.SimUart.Bit_n++;
	}
	else
	{
		HAL_GPIO_WritePin(SystemState.SimUart.GPIOx,SystemState.SimUart.GPIO_Pin,GPIO_PIN_SET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance==SystemState.htim->Instance)
		{
				SystemState.SystemTime_ms++;
		}
		if(htim->Instance==SystemState.SimUart.htim->Instance)
		{
			SimulateUARTInISR();
		}
}

