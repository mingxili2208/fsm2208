/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_CONTROL_H
#define __AX_CONTROL_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//机器人各种控制方式处理文件
void AX_CTL_Ps2(void);     //PS2手柄控制
void AX_CTL_App(void);     //APP控制
void AX_CTL_RemoteSbus(void);  //SBUS航模遥控器控制
void FSM_CTL_NRF(void);  		//NRF24L01 Wireless controller control
void FSM_CTL_A69(void);			//A69 Wireless controller control

#endif

/******************* (C) 版权 2023 XTARK **************************************/
