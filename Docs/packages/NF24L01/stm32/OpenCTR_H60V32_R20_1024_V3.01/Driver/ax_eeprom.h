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
  * @内  容  EEPROM存储驱动头文件 
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_EEPROM_H
#define __AX_EEPROM_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//OpenCTR接口函数
void AX_EEPROM_Init(void);    //初始化
uint8_t AX_EEPROM_ReadOneByte(uint16_t addr);  //读取一个字节
void AX_EEPROM_WriteOneByte(uint16_t addr,uint8_t data);  //写一个字节
void AX_EEPROM_Read(uint16_t addr, uint8_t *buf, uint16_t num);  //读取数据
void AX_EEPROM_Write(uint16_t addr, uint8_t *buf, uint16_t num);  //写入数据


#endif

/******************* (C) 版权 2023 XTARK **************************************/
