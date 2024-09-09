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
  * @内  容  按键检测
  *
  ******************************************************************************
  * @说  明
  *
  * 1.获取键值，。
  * 
  ******************************************************************************
  */

#include "ax_key.h"
#include "ax_delay.h"
#include "ax_sys.h"

/**
  * @简  述  KEY 按键初始化
  * @参  数  无
  * @返回值  无
  */
void AX_KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	//GPIO配置 
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE); //启动GPIO时钟 
	

	//配置模式选择S1 GPIO  输入模式
	GPIO_SetBits(GPIOE,  GPIO_Pin_0);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
}

/**
  * @简  述  KEY 获取按键值
  * @参  数  无
  * @返回值  按键值，按键按下为1，抬起为0
  */
uint8_t AX_KEY_Scan(void)
{
	if(PEin(0) == 0)		
	{
		return 1;
	}
	
	return 0;
}



/******************* (C) 版权 2023 XTARK **************************************/
