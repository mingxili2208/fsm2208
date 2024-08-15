/**			                                                    
		   ____                    _____ _______ _____       XTARK@塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @内  容  VIN输入电压检测
  *
  ******************************************************************************
  * @说  明
  *
  * 1.使用ADC2对VIN输入电压采样，查询方式
	* 2.输入电压检测功能，可实现对电池电量的评估
  * 
  ******************************************************************************
  */

#include "ax_vin.h"

#define ADC_REVISE  99.6

/**
  * @简  述  VIN 输入电压检测初始化
  * @参  数  无
  * @返回值  无
  */
void AX_VIN_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

    //使能ADC2和GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 

    //配置PB1为模拟输入(ADC Channel10) 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	 
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);
	
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
    ADC_CommonInit(&ADC_CommonInitStructure);
	
    //配置ADC2
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;	
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	
    ADC_InitStructure.ADC_NbrOfConversion = 1; 
    ADC_Init(ADC2, &ADC_InitStructure);

    //使能ADC2
    ADC_Cmd(ADC2, ENABLE);
	
	//ADC2 调整通道配置 
    ADC_RegularChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_480Cycles);	
	
}

/**
  * @简  述  VIN 获得输入电压
  * @参  数  无	  
  * @返回值  电压值，扩大100倍。例如7.2V输出为720。
  */
uint16_t AX_VIN_GetVol_X100(void)
{
    uint16_t Input_Vol,temp;
	
	ADC_SoftwareStartConv(ADC2);   //使能指定的ADC2的, 软件转换启动功能 
	
    while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));//等待转换结束 
	
	temp = ADC_GetConversionValue(ADC2);
	
	Input_Vol = (uint16_t)((3.3*11.0*ADC_REVISE * temp)/4095);	//分压比例1/11
	
	return Input_Vol;
	
}


/******************* (C) 版权 2023 XTARK **************************************/
