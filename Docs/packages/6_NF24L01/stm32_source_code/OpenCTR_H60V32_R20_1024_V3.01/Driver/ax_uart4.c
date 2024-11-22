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
  * @内  容  蓝牙APP串口通信
  *
  ******************************************************************************
  * @说  明
  *
  * 1.USB串口通信，printf函数已定向到该串口，可以输出调试信息
  * 2.可以使用具有X-Protocol协议进行数据发送。
  * 3.开启UART的串口数据接收功能，使用中断方式，X-Protocol协议通信
  * 4.可通过AX_UART_GetRxData()函数判断是否有数据接收
  *
  * X-Protocol协议介绍（变帧长）
  * 帧定义：AA 55 | 0B  | 01  | 03 E8 FC 18 00 0A | 14
  *        帧头   帧长   帧码  数据                校验和
  * 帧  头：双帧头，抗干扰强
  * 帧  长：根据数据长度设定
  * 帧  码：用户根据功能设定，标识帧的唯一性
  * 数  据：高位在前，长度可变，内容自由组合8位，16位，32位数据
  * 校验和：前面数据累加和的低8位
  * 帧示例：( AA 55 0B 01 03 E8 FC 18 00 0A 14 ) 内容：1000，-1000,10,
  * 
  ******************************************************************************
  */

#include "ax_uart4.h"
#include <stdio.h>

#include "ax_robot.h"

static uint8_t uart4_rx_ok = 0; //接收成功标志
static uint8_t uart4_rx_con = 0;       //接收计数器
static uint8_t uart4_rx_checksum;    //帧头部分校验和
static uint8_t uart4_rx_buf[40];     //接收缓冲，数据内容小于等于32Byte
static uint8_t uart4_tx_buf[40];     //发送缓冲

/**
  * @简  述  UART   串口初始化
  * @参  数  baud： 波特率设置
  * @返回值	 无
  */
void AX_UART4_Init(uint32_t baud)
{

	GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef  NVIC_InitStructure;


	/* 串口USART配置 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	
	//USART对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); 

	//USART 端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

	//USART参数配置
	USART_InitStructure.USART_BaudRate = baud;    //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);

	//USART使能
	USART_Cmd(UART4, ENABLE); 
	
	//开启串口接收中断
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断
	
    //USART3 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	
	
}

/**
  * @简  述  UART 串口中断服务函数
  * @参  数  无 
  * @返回值  无
  */
void UART4_IRQHandler(void)
{
	uint8_t Res;
	
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收中断
	{
		Res =USART_ReceiveData(UART4);	
		
		if(uart4_rx_con < 3)    //==接收帧头 + 长度
		{
			if(uart4_rx_con == 0)  //接收帧头1 0xAA
			{
				if(Res == 0xAA)
				{
					uart4_rx_buf[0] = Res;
					uart4_rx_con = 1;					
				}
				else
				{
					
				}
			}else if(uart4_rx_con == 1) //接收帧头2 0x55
			{
				if(Res == 0x55)
				{
					uart4_rx_buf[1] = Res;
					uart4_rx_con = 2;
				}
				else
				{
					uart4_rx_con = 0;						
				}				
			}
			else  //接收数据长度
			{
				uart4_rx_buf[2] = Res;
				uart4_rx_con = 3;
				uart4_rx_checksum = (0xAA+0x55) + Res;	//计算校验和
			}
		}
		else    //==接收数据
		{
			if(uart4_rx_con < (uart4_rx_buf[2]-1) )
			{
				uart4_rx_buf[uart4_rx_con] = Res;
				uart4_rx_con++;
				uart4_rx_checksum = uart4_rx_checksum + Res;					
			}
			else    //判断最后1位
			{
				//接收完成，恢复初始状态
				uart4_rx_con = 0;	
				
				//数据校验
				if( Res == uart4_rx_checksum )  //校验正确
				{	

					//数据接收OK
					uart4_rx_ok = 1;	
					
					//手机APP完成蓝牙连接
					if(uart4_rx_buf[3] == ID_BLERX_CM)
					{
						if(uart4_rx_buf[4] == 0x55)
						{
							//切换到手机APP控制
							ax_control_mode = CTL_APP;
							
							//执行蜂鸣器鸣叫提示
							ax_beep_ring = BEEP_SHORT;
						}
					}
					
					//printf("ID = %d \r\n",uart4_rx_buf[4],uart4_rx_buf[4],uart4_rx_buf[4]);
				}
			}
		}
		
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	} 
}


/**
  * @简  述  获取接收的数据
  * @参  数  *pbuf：接收数据指针,第1个字节为帧编码，后面为数据
  * @返回值	 0-无数据接收，other-需要读取的数据字节个数
  */
uint8_t AX_UART4_GetData(uint8_t *pbuf)
{
	uint8_t cnt,i;
	
	if(uart4_rx_ok != 0)
	{
		cnt = uart4_rx_buf[2]-4;
		
		for(i=0; i<cnt; i++)
		{
			*(pbuf+i) = uart4_rx_buf[3+i];
		}
		
		uart4_rx_ok = 0;
		return cnt;
	}
	else
	{
		return 0;
	}	
}

/**
  * @简  述  UART 发送数据（X-Protocol协议）
  * @参  数  *pbuf：发送数据指针
  *          len：发送数据长度个数，≤27 (32-5)
  *          num：帧号，帧编码
  * @返回值	 无
  */
void AX_UART4_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num)
{
	uint8_t i,cnt;	
    uint8_t tx_checksum = 0;//发送校验和
	
	if(len <= 32)
	{
		/******获取数据******/
		uart4_tx_buf[0] = 0xAA;    //帧头
		uart4_tx_buf[1] = 0x55;    //
		uart4_tx_buf[2] = len+5;  //根据输出长度计算帧长度
		uart4_tx_buf[3] = num;    //帧编码
		
		for(i=0; i<len; i++)
		{
			uart4_tx_buf[4+i] = *(pbuf+i);
		}
		
		/******计算校验和******/	
		cnt = 4+len;
		for(i=0; i<cnt; i++)
		{
			tx_checksum = tx_checksum + uart4_tx_buf[i];
		}
		uart4_tx_buf[i] = tx_checksum;
		
		
		/******发送数据******/	
		cnt = 5+len;
		
		//查询传输方式
		for(i=0; i<cnt; i++)
		{
			USART_SendData(UART4, uart4_tx_buf[i]);
			while(USART_GetFlagStatus(UART4,USART_FLAG_TC) != SET);
		}
	}
}



/******************* (C) 版权 2023 XTARK **************************************/
