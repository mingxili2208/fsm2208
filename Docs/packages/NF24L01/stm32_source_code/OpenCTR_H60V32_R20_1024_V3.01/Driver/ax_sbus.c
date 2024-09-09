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
  * @内  容  SBUS航模遥控器驱动程序 
  *
  ******************************************************************************
  */

#include "ax_sbus.h"
#include <stdio.h>

#include "ax_robot.h"


#define SBUS_RECV_MAX    25
#define SBUS_START       0x0F
#define SBUS_END         0x00

static uint8_t  uart_sbus_rx_ok = 0;      //接收成功标志
static uint8_t  uart_sbus_rx_con=0;       //接收计数器
static uint8_t  uart_sbus_rx_buf[40];     //接收缓冲，数据内容小于等于32Byte

//启动判断
static uint16_t sbus_buf[2];             //通道数据

/**
  * @简  述  SBUS串口初始化
  * @参  数  无
  * @返回值	 无
  */
void AX_SBUS_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;

	/* 串口USART配置 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	
	//USART对应引脚复用映射
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2, GPIO_AF_UART5); 

	//USART 端口配置	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOD,&GPIO_InitStructure); 

	//USART参数配置
	USART_InitStructure.USART_BaudRate = 100000;    //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_Init(UART5, &USART_InitStructure);
	
    //USART3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	
	
	//开启串口接收中断
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
	
	//USART使能
	USART_Cmd(UART5, ENABLE); 
	
}

/**
  * @简  述  UART 串口中断服务函数
  * @参  数  无 
  * @返回值  无
  */
void UART5_IRQHandler(void)
{
	uint8_t Res;
	
	//接收中断
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)  
	{
		Res = USART_ReceiveData(UART5);	
		
		if(uart_sbus_rx_con == 0)
		{
			//检测数据开始字节
			if(Res == SBUS_START)
			{
				uart_sbus_rx_buf[uart_sbus_rx_con] = Res;
				
				uart_sbus_rx_con++; 
			}			
		}
		else
		{
			//检测数据结束字节
			if((uart_sbus_rx_con >= (SBUS_RECV_MAX-1)) && (Res ==  SBUS_END))
			{
				//一帧数据接收完毕
				uart_sbus_rx_ok = 1;
				
				if(ax_control_mode != CTL_RMS)
				{
					//获取通道4和通道8数据
					sbus_buf[0] = ((int16_t)uart_sbus_rx_buf[ 3] >> 6 | ((int16_t)uart_sbus_rx_buf[ 4] << 2 )  | (int16_t)uart_sbus_rx_buf[ 5] << 10 ) & 0x07FF;
					sbus_buf[1] = ((int16_t)uart_sbus_rx_buf[10] >> 5 | ((int16_t)uart_sbus_rx_buf[11] << 3 )) & 0x07FF;
					
					//右侧摇杆（通道2）拨到上端，并且SWD-8开关（通道8）拨到最下端，进入航模遥控器控制模式
					if((sbus_buf[0] > 1500) && (sbus_buf[1] > 1500))
					{
						//切换到航模遥控器SBUS控制
						ax_control_mode = CTL_RMS;
						
						//执行蜂鸣器鸣叫提示
						ax_beep_ring = BEEP_SHORT;
					}					
				}
				
				//复位接收计数器
				uart_sbus_rx_con = 0;
			}
			else
			{
				uart_sbus_rx_buf[uart_sbus_rx_con] = Res;
				
				uart_sbus_rx_con++; 
			}
		}
		
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
	}
}

/**
  * @简  述  获取接收的数据
  * @参  数  *pbuf：通道数据
  * @返回值	 0-无数据接收，1-有数据
  */
uint8_t AX_SBUS_GetRxData(uint16_t *pbuf)
{

	if(uart_sbus_rx_ok != 0)
	{
		//计算遥控器通道数据
		*(pbuf+0) = ((int16_t)uart_sbus_rx_buf[ 1] >> 0 | ((int16_t)uart_sbus_rx_buf[ 2] << 8 )) & 0x07FF;
		*(pbuf+1) = ((int16_t)uart_sbus_rx_buf[ 2] >> 3 | ((int16_t)uart_sbus_rx_buf[ 3] << 5 )) & 0x07FF;
		*(pbuf+2) = ((int16_t)uart_sbus_rx_buf[ 3] >> 6 | ((int16_t)uart_sbus_rx_buf[ 4] << 2 )  | (int16_t)uart_sbus_rx_buf[ 5] << 10 ) & 0x07FF;
		*(pbuf+3) = ((int16_t)uart_sbus_rx_buf[ 5] >> 1 | ((int16_t)uart_sbus_rx_buf[ 6] << 7 )) & 0x07FF;
		*(pbuf+4) = ((int16_t)uart_sbus_rx_buf[ 6] >> 4 | ((int16_t)uart_sbus_rx_buf[ 7] << 4 )) & 0x07FF;
		*(pbuf+5) = ((int16_t)uart_sbus_rx_buf[ 7] >> 7 | ((int16_t)uart_sbus_rx_buf[ 8] << 1 )  | (int16_t)uart_sbus_rx_buf[9] <<  9 ) & 0x07FF;
		*(pbuf+6) = ((int16_t)uart_sbus_rx_buf[9] >> 2 | ((int16_t)uart_sbus_rx_buf[10] << 6 )) & 0x07FF;
		*(pbuf+7) = ((int16_t)uart_sbus_rx_buf[10] >> 5 | ((int16_t)uart_sbus_rx_buf[11] << 3 )) & 0x07FF;
		
//		*(pbuf+8) = ((int16_t)uart_sbus_rx_buf[12] << 0 | ((int16_t)uart_sbus_rx_buf[13] << 8 )) & 0x07FF;
//		*(pbuf+9) = ((int16_t)uart_sbus_rx_buf[13] >> 3 | ((int16_t)uart_sbus_rx_buf[14] << 5 )) & 0x07FF;
//		*(pbuf+10) = ((int16_t)uart_sbus_rx_buf[14] >> 6 | ((int16_t)uart_sbus_rx_buf[15] << 2 )  | (int16_t)uart_sbus_rx_buf[16] << 10 ) & 0x07FF;
//		*(pbuf+11) = ((int16_t)uart_sbus_rx_buf[16] >> 1 | ((int16_t)uart_sbus_rx_buf[17] << 7 )) & 0x07FF;
//		*(pbuf+12) = ((int16_t)uart_sbus_rx_buf[17] >> 4 | ((int16_t)uart_sbus_rx_buf[18] << 4 )) & 0x07FF;
//		*(pbuf+13) = ((int16_t)uart_sbus_rx_buf[18] >> 7 | ((int16_t)uart_sbus_rx_buf[19] << 1 )  | (int16_t)uart_sbus_rx_buf[20] <<  9 ) & 0x07FF;
//		*(pbuf+14) = ((int16_t)uart_sbus_rx_buf[20] >> 2 | ((int16_t)uart_sbus_rx_buf[21] << 6 )) & 0x07FF;
//		*(pbuf+15) = ((int16_t)uart_sbus_rx_buf[21] >> 5 | ((int16_t)uart_sbus_rx_buf[22] << 3 )) & 0x07FF;
		
		//复位标志位
		uart_sbus_rx_ok = 0;
		return 1;
	}
	else
	{
		return 0;
	}	
}

/******************* (C) 版权 2023 XTARK **************************************/













