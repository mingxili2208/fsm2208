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
  * @内  容  拨码开关检测
  *
  ******************************************************************************
  * @说  明
  *
  * 
  * 
  ******************************************************************************
  */

#include "ax_switch.h"

/**
  * @简  述  拨码开关 初始化
  * @参  数  无
  * @返回值  无
  */
void AX_SW_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	// GPIO配置
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE); //启动GPIO时钟 
	

	//配置模式选择S1 GPIO  输入模式
	GPIO_SetBits(GPIOB,  GPIO_Pin_0);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//配置模式选择S2 GPIO  输入模式 
	GPIO_SetBits(GPIOB,  GPIO_Pin_1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * @简  述  SW1获得选择状态
  * @参  数  无	  
  * @返回值  编码开关状态
  *			 1 - 开关拨动到ON侧
  *			 0 - 开关拨动到ON对侧
  */
uint8_t AX_SW_GetS1(void)
{
   	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0))
	return 0;
	else
	return 1;
}

/**
  * @简  述  SW2获得选择状态
  * @参  数  无	  
  * @返回值  编码开关状态
  *			 1 - 开关拨动到ON侧
  *			 0 - 开关拨动到ON对侧
  */
uint8_t AX_SW_GetS2(void)
{
   	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1))
	return 0;
	else
	return 1;
}


/**
  * @简  述  SW获得选择状态
  * @说  明	 开关拨动到ON侧为1，对侧为0。
  * @参  数  无	  
  * @返回值  编码状态
  *			SW1(PA8)  SW2(PE10)    返回值
  *			 0         0             0  
  *          0         1             1  
  *          1         0             2
  *          1         1             3   
  */
uint8_t AX_SW_GetS12(void)
{
    uint8_t sw = 0;

	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1)) 	
	{sw <<= 1;}
	else
	{sw=sw+1; sw<<=1;}

	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0))
	{;}
	else
	{sw=sw+1;}

	return sw;
}

/******************* (C) 版权 2023 XTARK **************************************/
