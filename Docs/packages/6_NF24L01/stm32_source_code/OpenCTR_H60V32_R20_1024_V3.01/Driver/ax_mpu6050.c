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
  * @内  容  MPU6050 操作
  *
  ******************************************************************************
  * @说  明
  *
  * 1.MPU6050三轴加速度传感器、三轴陀螺仪
  * 2.IIC通信采用IO口模拟方式
  *
  ******************************************************************************
  */

#include "ax_mpu6050.h" 
#include "ax_sys.h"
#include "ax_delay.h"

//IO方向设置
#define SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //输出模式

//IO操作函数	 
#define IIC_SCL    PBout(6) //SCL
#define IIC_SDA    PBout(7) //SDA	 
#define READ_SDA   PBin(7)  //输入SDA 

//函数定义
static void MPU6050_WriteRegister(uint8_t reg_address, uint8_t data);
static void MPU6050_ReadRegister(uint8_t reg_address, uint8_t *pdata, uint16_t len);

/**
  * @简  述  MPU6050传感器初始化
  * @参  数  无	  
  * @返回值  无
  */
void AX_MPU6050_Init(void)
{	
	//初始化I2C2
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟

	//SCL设置为推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
	//SDA设置为开漏输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
	
    IIC_SCL=1;
    IIC_SDA=1;
	
	//复位唤醒MPU6050
    MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x80);      //复位
	AX_Delayus(100000);  	//等待传感器稳定
	MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x00);      //解除休眠状态
	
	//配置MPU6050寄存器  
	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,0x18);      //陀螺仪量程 默认2000deg/s
    MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,0x00);     //加速计量程 默认2g	
	MPU6050_WriteRegister(MPU6050_INT_EN_REG,0x00);      //关闭中断
	MPU6050_WriteRegister(MPU6050_FIFO_EN_REG,0x00);       //关闭FIFO
	
	MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x01);      //设置CLKSEL,PLL X轴为参考
	MPU6050_WriteRegister(MPU6050_PWR_MGMT2_REG,0x00); 	    //加速度与陀螺仪都工作	

}


/**
  * @简  述  MPU6050设置加速度量程 
  * @参  数  range： 加速度量程2g、4g、8g、16g
  *          可设置的加速度量程ACC_RANGE_2G、ACC_RANGE_4G、ACC_RANGE_8G、ACC_RANGE_16G
  * @返回值  无
  */
void AX_MPU6050_SetAccRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,range<<3);
}


/**
  * @简  述  MPU6050设置陀螺仪量程
  * @参  数  range 陀螺仪量程250°/S、500°/S、1000°/S、2000°/S
  *          可设置的陀螺仪量程GYRO_RANGE_250、GYRO_RANGE_500、GYRO_RANGE_1000、GYRO_RANGE_2000	
  * @返回值  无
  */
void AX_MPU6050_SetGyroRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,range<<3);
}

/**
  * @简  述  MPU6050设置陀螺仪采样率
  * @参  数  smplrate 陀螺仪采样率，范围10~1000Hz	  
  * @返回值  无
  */
void AX_MPU6050_SetGyroSmplRate(uint16_t smplrate)
{	
	if(smplrate>1000)
		smplrate = 1000;
	if(smplrate<10)
		smplrate = 10;
	
	MPU6050_WriteRegister(MPU6050_SAMPLE_RATE_REG,(uint8_t)(1000/smplrate -1));	
}

/**
  * @简  述  MPU6050设置低通滤波器带宽
  * @参  数  bandwidth 低通滤波器带宽
  *          可设置的带宽： DLPF_ACC184_GYRO188、DLPF_ACC94_GYRO98、DLPF_ACC44_GYRO42、
  *                        DLPF_ACC21_GYRO20、DLPF_ACC10_GYRO10、DLPF_ACC5_GYRO5
  * @返回值  无
  */
void AX_MPU6050_SetDLPF(uint8_t bandwidth)
{
	MPU6050_WriteRegister(MPU6050_CFG_REG,bandwidth);
}

/**
  * @简  述  MPU6050获取传感器温度值
  * @参  数  无	  
  * @返回值  传感器温度值。
  */
float AX_MPU6050_GetTempValue(void)
{	
	uint8_t buf[2];
	int16_t tmp;

	MPU6050_ReadRegister(MPU6050_TEMP_OUTH_REG,buf,2);

	tmp = (buf[0]<<8)| buf[1];
	
	return ( 36.53f + ((double)tmp/340.0f) );	
}


/**
  * @简  述  MPU6050获取三轴加速度寄存器输出值
  * @参  数  pbuf：读取的数据缓冲区指针 
  * @返回值  无
  */
void AX_MPU6050_GetAccData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	//读取加速度数据
	MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG,buf,6);
	
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}

/**
  * @简  述  MPU6050获取三轴陀螺仪寄存器输出值
  * @参  数  pbuf：读取的数据缓冲区指针 
  * @返回值  无
  */
void AX_MPU6050_GetGyroData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	//读取陀螺仪数据
	MPU6050_ReadRegister(MPU6050_GYRO_XOUTH_REG,buf,6);
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}


//--------------------------I2C 初始化操作函数-----------------------------------------

/**
  * @简  述  产生IIC起始信号
  * @参  数  无
  * @返回值  返回状态
  */
static uint8_t IIC_Start(void)
{
	//SDA线输出
	SDA_OUT();     
	
	//START信号: 当SCL为高时, SDA从高变成低, 表示起始信号
	IIC_SDA=1;
	
	if(!READ_SDA)
	{
		return 0;	
	}
		
	IIC_SCL=1;
	AX_Delayus(1);
	
 	IIC_SDA=0;
	if(READ_SDA)
		return 0;
	
	AX_Delayus(1);
	
	//钳住I2C总线，准备发送或接收数据 
	IIC_SCL=0;
	
	return 1;
}

/**
  * @简  述  产生IIC停止信号
  * @参  数  无
  * @返回值  无
  */	  
static void IIC_Stop(void)
{
	//SDA线输出
	SDA_OUT(); 
	
	//STOP信号: 当SCL为高时, SDA从低变成高, 表示停止信号
	IIC_SCL=0;
	IIC_SDA=0;
 	AX_Delayus(1);
	
	//发送I2C总线结束信号
	IIC_SCL=1; 
	IIC_SDA=1;
	AX_Delayus(1);							   	
}

/**
  * @简  述  等待应答信号到来
  * @参  数  无
  * @返回值  0，接收应答成功  1，接收应答失败
  */
static uint8_t  IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	
	//SDA设置为输入  
	SDA_IN();     
	
	//主机释放SDA线(此时外部器件可以拉低SDA线)
	IIC_SDA=1;
	AX_Delayus(1);	
	
	//SCL=1, 此时从机可以返回ACK
	IIC_SCL=1;
	AX_Delayus(1);	
	
	//等待应答
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 0;
		}
		
	    //AX_Delayus(1);
	}
	
	//SCL=0, 结束ACK检查	
	IIC_SCL=0;
	AX_Delayus(1);
	
	return 1;  
} 

/**
  * @简  述  产生ACK应答产生IIC起始信号
  * @参  数  无
  * @返回值  无
  */
static void IIC_Ack(void)
{
	IIC_SCL=0;
	
	//SDA线输出
	SDA_OUT();
	
	IIC_SDA=0;
	AX_Delayus(1);
	
	IIC_SCL=1;
	AX_Delayus(1);
	IIC_SCL=0;
}

/**
  * @简  述  产生NACK应答
  * @参  数  无
  * @返回值  无
  */	    
static void IIC_NAck(void)
{
	IIC_SCL=0;
	
	//SDA线输出
	SDA_OUT();
	
	IIC_SDA=1;
	AX_Delayus(1);
	
	IIC_SCL=1;
	AX_Delayus(1);
	IIC_SCL=0;
}

/**
  * @简  述  IIC发送一个字节,返回从机有无应答
  * @参  数  data: 要发送的数据
  * @返回值  无
  */		  
static void IIC_Send_Byte(uint8_t data)
{                        
    uint8_t t;   
	
	//SDA线输出
	SDA_OUT();

    //拉低时钟开始数据传输
    IIC_SCL=0;
	
    for(t=0; t<8; t++)
    {   
		//高位先发送		
        IIC_SDA=(data&0x80)>>7;
  
		AX_Delayus(1);   
		IIC_SCL=1;
		AX_Delayus(1); 
		IIC_SCL=0;	
		AX_Delayus(1);
		
	    //左移1位,用于下一次发送		
		data<<=1; 
    }	 
} 	 
  
/**
  * @简  述  IIC读取一个字节
  * @参  数  ack:  ack=1时，发送ack; ack=0时，发送nack
  * @返回值  接收到的数据
  */
static uint8_t IIC_Read_Byte(uint8_t ack)
{
	unsigned char i,receive=0;
	
	//SDA设置为输入
	SDA_IN();
	
	//接收1个字节数据
    for(i=0;i<8;i++ )
	{
		//高位先输出,所以先收到的数据位要左移
		receive<<=1;
		
        IIC_SCL=0; 
        AX_Delayus(2);
		IIC_SCL=1;
        
        if(READ_SDA)
		{
			receive++; 
		}
		
		AX_Delayus(2); 
    }	
	
    if (ack)
    {
        IIC_Ack();  //发送ACK
    }
    else
    {
        IIC_NAck();   //发送NACK
    }

    return receive;
}


/**
  * @简  述  写入指定地址指定长度数据
  * @参  数  dev_addr：设备地址
  *          reg_addr：寄存器地址
  *          len：数据长度
  *          data：数据内容
  * @返回值  返回结果
  */
static uint8_t IIC_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, const uint8_t *data)
{
	int i;
	
    if (!IIC_Start())
        return 1;
	
    IIC_Send_Byte(dev_addr << 1 );
	
    if (!IIC_Wait_Ack()) 
	{
        IIC_Stop();
        return 1;
    }
	
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
	
	for (i = 0; i < len; i++) 
	{
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) 
		{
            IIC_Stop();
            return 0;
        }
    }
	
    IIC_Stop();
    return 0;
}

/**
  * @简  述  读取指定地址指定长度数据
  * @参  数  dev_addr：设备地址
  *          reg_addr：寄存器地址
  *          len：数据长度
  *          data：数据内容
  * @返回值  返回结果
  */
static uint8_t IIC_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    if (!IIC_Start())
        return 1;
	
    IIC_Send_Byte(dev_addr << 1);
    if (!IIC_Wait_Ack()) 
	{
        IIC_Stop();
        return 1;
    }
	
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
	
    IIC_Start();
    IIC_Send_Byte((dev_addr << 1)+1);
    IIC_Wait_Ack();
    while (len) 
	{
        if (len == 1)
            *data = IIC_Read_Byte(0);
        else
            *data = IIC_Read_Byte(1);
		
        data++;
        len--;
    }
	
    IIC_Stop();
    return 0;
}

/**
  * @简  述  MPU6050写寄存器。
  * @参  数  reg_addr：寄存器地址
  *          pdata：数据内容
  *          len：数据长度
  * @返回值  无
  */
static void MPU6050_WriteRegister(uint8_t reg_address, uint8_t data)
{
	IIC_Write(MPU6050_ADDR,reg_address,1,&data);
}

/**
  * @简  述  MPU6050读寄存器。
  * @参  数  无	  
  * @返回值  无
  */
static void MPU6050_ReadRegister(uint8_t reg_address, uint8_t *pdata, uint16_t len)
{
	IIC_Read(MPU6050_ADDR,reg_address,len,pdata);
}


/******************* (C) 版权 2023 XTARK **************************************/
