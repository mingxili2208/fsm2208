/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：5.7
修改时间：2021-04-29

Company: WeiHong Co.Ltd
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update：2021-04-29

All rights reserved
***********************************************/
#include "usart4.h"
#include <string.h>

LidarPointTypedef4 Pack_Data4[12];/* 雷达接收的数据储存在这个变量之中 */
LidarPointTypedef4 Pack_sum4;     /* 输出结果储存 */
extern u16 distance_2,distance_3;
extern u8 flag ;
extern u16 receive_cnt3,receive_cnt4;//计算成功接收数据帧次数;
static const uint8_t CrcTable[256] =
{
0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};//用于crc校验的数组
/**************************************************************************
Function: Usart3 initialization
Input   : bound:Baud rate
Output  : none
函数功能：串口2初始化
入口参数：bound:波特率
返回  值：无
**************************************************************************/
void uart4_init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;        
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE );
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE );
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;          //USART2 TX；
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;    //复用推挽输出；
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);             //端口A；
	    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;               //USART2 RX；
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入；
	GPIO_Init(GPIOC, &GPIO_InitStructure);                  //端口A；
	
	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	//USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART4, &USART_InitStructure);     //初始化串口2
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(UART4, ENABLE);                    //使能串口2 
}

/**************************************************************************
Function: Receive interrupt function
Input   : none
Output  : none
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
void UART4_IRQHandler(void)
{	
	static u8 state4 = 0;			//状态位	
	static u8 crc4 = 0;				//校验和
	static u8 cnt4 = 0;				//用于一帧12个点的计数
	static u8 PACK_FLAG4 = 0;  //命令标志位
	static u8 data_len4  = 0;  //数据长度
	static u32 timestamp4 = 0; //时间戳
	static u8 state_flag4 = 1; //转入数据接收标志位
	u8 temp_data;
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET) //接收到数据
	{	  
		
		temp_data=USART_ReceiveData(UART4); 
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);
	    if(state4< 4) 																					 /* 起始符验证 前4个数据均为0xAA */
				{                                          
						if(temp_data == HEADER) state4 ++;
						else state4 = 0;
				}
				else if(state4<10&&state4>3)
				{
						switch(state4)
						{
								case 4:   
									if(temp_data == device_address)              /* 设备地址验证  */
									{							
													state4 ++;
													crc4 = crc4 + temp_data;									
													break;        
									} 
									else state4 = 0,crc4 = 0;
								case 5:   
									if(temp_data == PACK_GET_DISTANCE)					 /* 获取测量数据命令 */
									{  
													PACK_FLAG4 = PACK_GET_DISTANCE;
													state4 ++;	
													crc4 = crc4 + temp_data;	
													break;									
									}		 

									else if(temp_data == PACK_RESET_SYSTEM) 		 /* 复位命令 */
									{
													PACK_FLAG4 = PACK_RESET_SYSTEM;
													state4 ++; 
													crc4 = crc4 + temp_data;	
													break;	
									}
									else if(temp_data == PACK_STOP)							 /* 停止测量数据传输命令 */
									{ 
													PACK_FLAG4 = PACK_STOP;
													state4 ++; 
													crc4 = crc4 + temp_data;	
													break;
									}
									else if(temp_data == PACK_ACK)							 /* 应答码命令 */
									{  
													PACK_FLAG4 = PACK_ACK;
													state4 ++;
													crc4 = crc4 + temp_data;	
													break;
									}			 				 
									else if(temp_data == PACK_VERSION)					 /* 获取传感器信息命令 */
									{ 
													PACK_FLAG4 = PACK_VERSION;
													state4 ++;
													crc4 = crc4 + temp_data;	   	     
													break;
									}
									else state4 = 0,crc4 = 0;
								case 6: if(temp_data == chunk_offset)          /* 偏移地址 */
												{  
													state4 ++;
													crc4 = crc4 + temp_data;
													break; 	  
												}	
												else state4 = 0,crc4 = 0;
								case 7: if(temp_data == chunk_offset)
												{  
													state4 ++;
													crc4 = crc4 + temp_data;
													break;
												}
												else state4 = 0,crc4 = 0;
								case 8: 
										data_len4 = (u16)temp_data;								 /* 数据长度低八位 */
										state4 ++; 
										crc4 = crc4 + temp_data;
										break;																			 
								case 9: 
										data_len4 = data_len4 + ((u16)temp_data<<8); 			 /* 数据长度高八位 */
										state4 ++;
										crc4 = crc4 + temp_data;
										break; 
								default: break;
						}
				}
				else if(state4 == 10 ) state_flag4 = 0;                    /*由switch跳出来时state为10，但temp_data仍为距离长度高八位数据，需跳过一次中断*/
				if(PACK_FLAG4 == PACK_GET_DISTANCE&&state_flag4 == 0)      /* 获取一帧数据并校验 */
				{
						if(state4>9)
						{
								if(state4<190)
								{
										static u8 state_num;
										state_num = (state4-10)%15;
										switch(state_num)
										{
												case 0: 
													Pack_Data4[cnt4].distance4 = (u16)temp_data ;				 /* 距离数据低八位 */
													crc4 = crc4 + temp_data;
													state4++;
													break;        
												case 1: 
													Pack_Data4[cnt4].distance4 = ((u16)temp_data<<8) + Pack_Data4[cnt4].distance4;	 /* 距离数据 */
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 2:
													Pack_Data4[cnt4].noise4 = (u16)temp_data;				 /* 环境噪音低八位 */
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 3:
													Pack_Data4[cnt4].noise4 = ((u16)temp_data<<8) + Pack_Data4[cnt4].noise4;				 /* 环境噪音 */
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 4:
													Pack_Data4[cnt4].peak4 = (u32)temp_data;				 										 /* 接受强度信息低八位 */
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 5:
													Pack_Data4[cnt4].peak4 = ((u32)temp_data<<8) + Pack_Data4[cnt4].peak4;
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 6:
													Pack_Data4[cnt4].peak4 = ((u32)temp_data<<16) + Pack_Data4[cnt4].peak4;	
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 7:
													Pack_Data4[cnt4].peak4 = ((u32)temp_data<<24) + Pack_Data4[cnt4].peak4;				    /* 接受强度信息 */
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 8:
													Pack_Data4[cnt4].confidence4 = temp_data;				 /* 置信度 */
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 9:
													Pack_Data4[cnt4].intg4 = (u32)temp_data;															/* 积分次数低八位 */
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 10:
													Pack_Data4[cnt4].intg4 = ((u32)temp_data<<8) + Pack_Data4[cnt4].intg4;
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 11:
													Pack_Data4[cnt4].intg4 = ((u32)temp_data<<16) + Pack_Data4[cnt4].intg4;
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 12:
													Pack_Data4[cnt4].intg4 = ((u32)temp_data<<24) + Pack_Data4[cnt4].intg4;				  	 /* 积分次数 */
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 13:
													Pack_Data4[cnt4].reftof4 = (int16_t)temp_data;				 								 /* 温度表征值低八位 */
													crc4 = crc4 + temp_data;
													state4++;
													break; 
												case 14:
													Pack_Data4[cnt4].reftof4 = ((int16_t)temp_data<<8) +Pack_Data4[cnt4].reftof4;			/* 温度表征值 */
													crc4 = crc4 + temp_data;
													state4++;
													cnt4++;							 /* 进入下一个测量点 */
													break; 
												default: break;
										}
							}
										/* 时间戳 */
										if(state4 == 190) timestamp4 = temp_data,state4++,crc4 = crc4 + temp_data;
										else if(state4 == 191) timestamp4 = ((u32)temp_data<<8) + timestamp4,state4++,crc4 = crc4 + temp_data; 
										else if(state4 == 192) timestamp4 = ((u32)temp_data<<16) + timestamp4,state4++,crc4 = crc4 + temp_data;
										else if(state4 == 193) timestamp4 = ((u32)temp_data<<24) + timestamp4,state4++,crc4 = crc4 + temp_data; 
										else if(state4==194)
										{
													if(temp_data == crc4)   /* 校验成功 */
													{
															data_process4();  	 /* 数据处理函数，完成一帧之后可进行数据处理 */
															receive_cnt4++;	 	 /* 输出接收到正确数据的次数 */
													}
													distance_3 = Pack_Data4[0].distance4;
													crc4 = 0;
													state4 = 0;
													state_flag4 = 1;
													cnt4 = 0; 							 /* 复位*/
										}
							
						}
				}

	}		
} 


void data_process4(void)/*数据处理函数，完成一帧之后可进行数据处理*/
{
		/* 计算距离 */
//		static u8 cnt = 0;
		u8 i;
		static u16 count = 0;
//		static u32 sum = 0;
		//LidarPointTypedef4 Pack_sum;
		for(i=0;i<12;i++)									/* 12个点取平均 */
		{
				if(Pack_Data4[i].distance4 != 0)  /* 去除0的点 */
				{
						count++;
						Pack_sum4.distance4 += Pack_Data4[i].distance4;
						Pack_sum4.noise4 += Pack_Data4[i].noise4;
						Pack_sum4.peak4 += Pack_Data4[i].peak4;
						Pack_sum4.confidence4 += Pack_Data4[i].confidence4;
						Pack_sum4.intg4 += Pack_Data4[i].intg4;
						Pack_sum4.reftof4 += Pack_Data4[i].reftof4;
				}
		}
		if(count !=0)
		{

					distance_3 = Pack_sum4.distance4/count;
//					noise = Pack_sum.noise4/count;
//					peak = Pack_sum.peak4/count;
//					confidence = Pack_sum.confidence4/count;
//					intg = Pack_sum.intg4/count;
//					reftof = Pack_sum.reftof4/count;
					Pack_sum4.distance4 = 0;
					Pack_sum4.noise4 = 0;
					Pack_sum4.peak4 = 0;
					Pack_sum4.confidence4 = 0;
					Pack_sum4.intg4 = 0;
					Pack_sum4.reftof4 = 0;
					count = 0;
		}
}





