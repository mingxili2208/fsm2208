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
  * @内  容  CAN通信例程
  *
  ******************************************************************************
  * @说  明
  *
  * CAN发送端配置说明:
  * 波特率：1000K 
  * 帧ID： 0x0181
  * 帧格式：数据帧
  * DLC：8字节
  * 接收帧协议：X高 | X低 | Y高 | Y低 | Z高 | Z低 | 预留 | 预留  
  * 
  ******************************************************************************
  */

#include "ax_can.h"

#include "ax_robot.h"

static CanTxMsg TxMessage;  //发送帧
static CanRxMsg RxMessage; //接收帧

/**
  * @简  述  CAN初始化
  * @说  明  波特率为1M（分频参数6，TSJW=1，TBS1=4，TBS2=2）
  * @返回值  无
  */
void AX_CAN_Init(void)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	NVIC_InitTypeDef  NVIC_InitStructure;
	CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

    //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能PORT时钟	                   											 
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1); 
	  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	   //非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	   //软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;    //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	   //禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	   //报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	   //优先级由报文标识符决定 
	
  	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;    //CAN工作模式设置为普通模式
  	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;	    //重新同步跳跃宽度1个时间单位，范围:1~3;
  	CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;        //时间段1为3个时间单位，范围:1~8;
  	CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;        //时间段2为2个时间单位，范围:1~16;
  	CAN_InitStructure.CAN_Prescaler=6;              //波特率分频器，设定时间单位长度，波特率=Fpclk1/((tbs1+tbs2+1)*brp),例42M/(6*(1+4+2))=1000K
  	CAN_Init(CAN1, &CAN_InitStructure);             //初始化CAN1 
    
    //配置过滤器
 	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;  //过滤器标识符的高16位值
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;  //过滤器标识符的低16位值
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;  //过滤器屏蔽标识符的高16位值
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;  //过滤器屏蔽标识符的低16位值		
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;  //指定过滤器为标识符屏蔽位模式
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;   //过滤器位宽为32
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;  //设定了指向过滤器的FIFO为0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;   //过滤器使能
  	CAN_FilterInit(&CAN_FilterInitStructure);  //滤波器初始化	
	
	//使能FIFO0消息挂号中断
    CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE); 
	
	//NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	
	
} 

/**
  * @简  述  CAN中断
  * @参  数  无
  * @返回值	 无
  */
void CAN1_RX0_IRQHandler(void)
{

	//接收FIFO0中的数据
    CAN_Receive(CAN1,CAN_FIFO0, &RxMessage); 
	
	//设置为ROS控制模式
	ax_control_mode = CTL_ROS;
	
	//消息解析
	if(RxMessage.StdId == ID_CANRX_VEL)  //速度数据
	{
		R_Vel.TG_IX = (int16_t)((RxMessage.Data[0]<<8) | RxMessage.Data[1]);
		R_Vel.TG_IY = (int16_t)((RxMessage.Data[2]<<8) | RxMessage.Data[3]);
		R_Vel.TG_IW = (int16_t)((RxMessage.Data[4]<<8) | RxMessage.Data[5]);
		
		//如果是阿克曼机器人，根据矢量速度计算前轮转向角
		#if (ROBOT_TYPE == ROBOT_AKM)
			ax_akm_angle = AX_AKM_WToAngle(R_Vel.TG_IX, R_Vel.TG_IW);
		#endif
	}
	
//	//查看接收的数据
//	printf("ID%02X L%02X DATA%02X %02X %02X %02X %02X %02X %02X %02X\r\n",RxMessage.StdId,RxMessage.DLC,
//	RxMessage.Data[0],RxMessage.Data[1],RxMessage.Data[2],RxMessage.Data[3],
//	RxMessage.Data[4],RxMessage.Data[5],RxMessage.Data[6],RxMessage.Data[7]);		
	
} 


/**
  * @简  述  CAN 发送标准帧数据
  * @参  数  *pbuf：发送数据指针
  *          len：发送数据长度个数，≤8
  *          id：帧号，帧编码范围0-0xFF
  * @返回值	 是否发送完成
  */
uint8_t AX_CAN_SendPacket(uint32_t id, uint32_t len, uint8_t *pbuf )
{
	uint8_t i;  

	//封装数据
	TxMessage.StdId=id;	  //标准标识符
	TxMessage.IDE=CAN_ID_STD;  //使用标准标识符
	TxMessage.RTR=CAN_RTR_DATA;  //为数据帧
	TxMessage.DLC=len;	 //数据长度,单位为字节

	for(i=0; i<len; i++)
	{
		TxMessage.Data[i] = *(pbuf+i);
	}

	//发送数据	
	return (CAN_Transmit(CAN1,&TxMessage)); 
}


/******************* (C) 版权 2023 XTARK **************************************/
