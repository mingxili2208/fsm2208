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
  * @内  容  EEPROM驱动程序 
  *
  ******************************************************************************
  * @说  明
  *
  * 
  ******************************************************************************
  */

#include "ax_eeprom.h"
#include "ax_sys.h"
#include "ax_delay.h"

//IO操作函数	 
#define  IIC_SCL    PEout(2) //SCL
#define  IIC_SDA    PEout(3) //SDA	 
#define  READ_SDA   PEin(3)  //输入SDA 

//IIC操作函数定义
static void IIC_Start(void);
static void IIC_Stop(void);
static uint8_t IIC_Wait_Ack(void);
static void IIC_Ack(void);
static void IIC_NAck(void);
static void IIC_Send_Byte(uint8_t data);
static uint8_t IIC_Read_Byte(uint8_t ack);

/**
  * @简  述  EEPROM初始化
  * @参  数  无	  
  * @返回值  无
  */
void AX_EEPROM_Init(void)
{
	//IIC初始化
	GPIO_InitTypeDef  GPIO_InitStructure;
	
    //使能GPIOB时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	//GPIO初始化设置 SCL
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化
	
	//GPIO初始化设置 SDA
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;  //开漏输出
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化

	//STOP信号
	IIC_SCL=1; 
	IIC_SDA=1;   
}

/**
  * @简  述  指定地址读出一个数据
  * @参  数  addr :开始的地址, 对24c02为0~255
  * @返回值  读到的数据
  */
uint8_t AX_EEPROM_ReadOneByte(uint16_t addr)
{				  
	uint8_t temp=0;	
	
	//发送起始信号
    IIC_Start();  
	
	//地址为1个低字节地址 + 占用器件地址的bit1~bit3位 用于表示高位地址, 最多11位地址
	//对于24C02, 其器件地址格式(8bit)为: 1  0  1  0  A2  A1  A0  R/W
	//R/W      : 读/写控制位 0,表示写; 1,表示读;
	//A0/A1/A2 : 对应器件的1,2,3引脚(只有24C01/02/04/8有这些脚)
	//a8/a9/a10: 对应存储整列的高位地址, 11bit地址最多可以表示2048个位置, 可以寻址24C16及以内的型号
	
	//发送器件 0xA0 + 高位a8/a9/a10地址,写数据
	IIC_Send_Byte(0XA0 + ((addr >> 8) << 1) );   	
	IIC_Wait_Ack();
	
	//发送低地址
    IIC_Send_Byte(addr%256);   
	IIC_Wait_Ack();	    
	
	//重新发送起始信号
    IIC_Start();  	
	
	//进入接收模式, IIC规定最低位是1, 表示读取	 	   
	IIC_Send_Byte(0XA1);       	   
	IIC_Wait_Ack();	
	
	//接收一个字节数据
    temp=IIC_Read_Byte(0);	
	
	//产生一个停止条件	
    IIC_Stop();                  
	
	return temp;
}


/**
  * @简  述  指定地址写入一个数据
  * @参  数  addr:开始的地址 对24c02为0~255
             data:要写入的数据
  * @返回值  读到的数据
  */
void AX_EEPROM_WriteOneByte(uint16_t addr,uint8_t data)
{	
	//发送起始信号	
    IIC_Start();  
	
	//发送器件 0xA0 + 高位a8/a9/a10地址,写数据
	IIC_Send_Byte(0XA0 + ((addr >> 8) << 1) );   	
	IIC_Wait_Ack();	   
	
	//发送低地址
    IIC_Send_Byte(addr%256);   
	IIC_Wait_Ack(); 

	//发送字节		
	IIC_Send_Byte(data);   
	IIC_Wait_Ack();  

	//产生一个停止条件
    IIC_Stop(); 
	
	//注意EEPROM 写入比较慢,必须等到10ms后再写下一个字节
	AX_Delayus(10000);	 
}


/**
  * @简  述  读出指定地址开始的指定个数的数据
  * @参  数  addr:开始的地址 对24c02为0~255
  *          buf:数据数组首地址
  *          num:要读出数据的个数
  * @返回值  无
  */
void AX_EEPROM_Read(uint16_t addr,uint8_t *buf, uint16_t num)
{
	while(num)
	{
		*buf ++= AX_EEPROM_ReadOneByte(addr++);	
		
		num--;
	}
}  


/**
  * @简  述  写入指定地址开始的指定个数的数据
  * @参  数  addr:开始的地址 对24c02为0~255
  *          buf:数据数组首地址
  *          num:要读出数据的个数
  * @返回值  无
  */
void AX_EEPROM_Write(uint16_t addr, uint8_t *buf, uint16_t num)
{
	while(num--)
	{
		AX_EEPROM_WriteOneByte(addr,*buf);
		
		addr++;
		buf++;
	}
} 



//--------------------------I2C操作函数-----------------------------------------

/**
  * @简  述  产生IIC起始信号
  * @参  数  无
  * @返回值  无
  */
static void IIC_Start(void)
{ 
	
    //START信号: 当SCL为高时, SDA从高变成低, 表示起始信号
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	AX_Delayus(2);

 	IIC_SDA=0;     
	AX_Delayus(2);
	
	//钳住I2C总线，准备发送或接收数据
	IIC_SCL=0;      
	AX_Delayus(2);
}


/**
  * @简  述  产生IIC停止信号
  * @参  数  无
  * @返回值  无
  */
static void IIC_Stop(void)
{
	//STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号
	IIC_SDA=0;      
 	AX_Delayus(2);
	
	IIC_SCL=1; 
	AX_Delayus(2);
	
	//发送I2C总线结束信号
	IIC_SDA=1;      
	AX_Delayus(2);							   	
}


/**
  * @简  述  等待应答信号到来
  * @参  数  无
  * @返回值  0，接收应答成功  1，接收应答失败
  */
static uint8_t IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	
	//主机释放SDA线(此时外部器件可以拉低SDA线)
	IIC_SDA=1;      
	AX_Delayus(2);	 
	
	//SCL=1, 此时从机可以返回ACK
	IIC_SCL=1;      
	AX_Delayus(2);	
	
	//等待应答
	while(READ_SDA)
	{
		ucErrTime++;
		
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	
	//SCL=0, 结束ACK检查	
	IIC_SCL=0;   
	AX_Delayus(2);
	
	return 0;  
} 

/**
  * @简  述  产生ACK应答产生IIC起始信号
  * @参  数  无
  * @返回值  无
  */
static void IIC_Ack(void)
{

	//SCL 0 -> 1  时 SDA = 0,表示应答
	IIC_SDA=0;
	AX_Delayus(2);	
	
	//产生一个时钟
	IIC_SCL=1;
	AX_Delayus(2);
	IIC_SCL=0;
	AX_Delayus(2);
	
	//主机释放SDA线
	IIC_SDA=1;
	AX_Delayus(2);

}
	
/**
  * @简  述  产生NACK应答
  * @参  数  无
  * @返回值  无
  */
static void IIC_NAck(void)
{
	//SCL 0 -> 1  时 SDA = 1,表示不应答
	IIC_SDA=1;
	AX_Delayus(2);
	
	//产生一个时钟
	IIC_SCL=1;
	AX_Delayus(2);
	IIC_SCL=0;
	AX_Delayus(2);
}	


/**
  * @简  述  IIC发送一个字节,返回从机有无应答
  * @参  数  data: 要发送的数据
  * @返回值  无
  */
static void IIC_Send_Byte(uint8_t data)
{                        
    uint8_t t;   
	   
    for(t=0; t<8; t++)
    {   
		//高位先发送
        IIC_SDA=(data&0x80)>>7;
		AX_Delayus(2);  

		//产生一个时钟
		IIC_SCL=1;
		AX_Delayus(2); 
		IIC_SCL=0;	
		
		//左移1位,用于下一次发送		
		data<<=1; 
    }	
	
	//发送完成, 主机释放SDA线
	IIC_SDA=1;
} 	 


/**
  * @简  述  IIC读取一个字节
  * @参  数  ack:  ack=1时，发送ack; ack=0时，发送nack
  * @返回值  接收到的数据
  */
static uint8_t IIC_Read_Byte(uint8_t ack)
{                        
    uint8_t i, receive = 0;

	//接收1个字节数据
    for (i = 0; i < 8; i++ )    /*  */
    {
		//高位先输出,所以先收到的数据位要左移
        receive <<= 1; 
		
        IIC_SCL = 1;
        AX_Delayus(2);

        if (READ_SDA)
        {
            receive++;
        }
		
        IIC_SCL = 0;
        AX_Delayus(2);
    }

    if (!ack)
    {
        IIC_NAck();     /* 发送nACK */
    }
    else
    {
        IIC_Ack();      /* 发送ACK */
    }

    return receive;
} 	

/******************* (C) 版权 2023 XTARK **************************************/
