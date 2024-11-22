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
  * @内  容  机器人轮子PID速度控制
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_SPEED_H
#define __AX_SPEED_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//电机PID闭环速度控制函数
int16_t AX_SPEED_PidCtlA(float spd_target, float spd_current);   //PID控制函数，电机A
int16_t AX_SPEED_PidCtlB(float spd_target, float spd_current);    //PID控制函数，电机B
int16_t AX_SPEED_PidCtlC(float spd_target, float spd_current);    //PID控制函数，电机C
int16_t AX_SPEED_PidCtlD(float spd_target, float spd_current);    //PID控制函数，电机D


#endif

/******************* (C) 版权 2023 XTARK **************************************/
