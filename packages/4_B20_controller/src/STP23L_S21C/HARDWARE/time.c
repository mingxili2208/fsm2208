#include "time.h"

void time5_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	
	TIM_TimeBaseInitStructure.TIM_Period = 49;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7199;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode =TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearFlag(TIM5,TIM_FLAG_Break);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM5,ENABLE);
}

void TIM5_IRQHandler(void)
{
	if(TIM_GetFlagStatus(TIM5,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM5,TIM_IT_Update);
		count++;
		LED_Flash(50);
		if(count==10)
		{
			count=0;
			Send_Flag=1;
		}
	}
}
