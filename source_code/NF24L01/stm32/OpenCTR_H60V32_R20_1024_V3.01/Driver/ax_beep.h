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
  * @内  容  BEEP蜂鸣器控制
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_BEEP_H
#define __AX_BEEP_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

// 接口函数
void AX_BEEP_Init(void);

// 蜂鸣器操作函数宏定义
#define AX_BEEP_Off()       GPIO_ResetBits(GPIOE, GPIO_Pin_1)       //蜂鸣器不鸣
#define AX_BEEP_On()		GPIO_SetBits(GPIOE, GPIO_Pin_1)     //蜂鸣器鸣叫
#define AX_BEEP_Toggle()    GPIO_WriteBit(GPIOE, GPIO_Pin_1, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)))    //蜂鸣器状态翻转

#endif 

/******************* (C) 版权 2023 XTARK **************************************/
