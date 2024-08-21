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
  * @日  期  2023-1-26
  * @内  容  OpenCTR B20控制器功能展示示例代码
  * 
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>   


#include "ax_sys.h"    //系统设置
#include "ax_delay.h"  //软件延时
#include "ax_led.h"    //LED灯控制
#include "ax_beep.h"   //蜂鸣器控制
#include "ax_uart1.h"  //调试串口
#include "ax_vin.h"    //输入电压检测
#include "ax_key.h"    //按键检测 
#include "ax_flash.h"  //FLASH读写

#include "ax_uart2.h"   //功能串口
#include "ax_uart3.h"   //功能串口
#include "ax_uart4.h"   //功能串口
#include "ax_uart5.h"   //功能串口

#include "ax_mpu6050.h" //IMU加速度陀螺仪测量
#include "ax_servo.h"   //舵机控制
#include "ax_motor.h"   //直流电机调速控制
#include "ax_encoder.h" //编码器控制



/******************************************************************************
      基础例程  清单
			
* LED闪烁，蜂鸣器，调试串口Printf输出例程
* VIN输入电压检测例程，简易电量计
* KEY按键检测测例程，软件消抖
* MPU6050数据采集例程
* 舵机控制例程
* 电机PWM速度控制例程
* 电机AB正交编码器例程
* UART2串口通信例程
* UART3串口通信例程
* UART4串口通信例程
* UART5串口通信例程
* FLASH读写例程

* 更多扩展例程，敬请期待

*******************************************************************************/



/******************************************************************************
例程名称：LED闪烁，蜂鸣器鸣叫，调试串口Printf输出
例程说明：本例程演示LED控制和调试串口Printf功能
*******************************************************************************/
int main(void)
{
	uint8_t i=0;
	
	//设置中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

	//JTAG口设置
	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 
	
	//软件延时初始化
	AX_DELAY_Init(); 	
	AX_LED_Init();  //LED初始化
	AX_BEEP_Init();  //蜂鸣器初始化

	//调试串口初始化
	AX_UART1_Init(115200); //调试串口
	printf("  \r\n"); //输出空格，CPUBUG
	
	//LED点亮0.5S
	AX_LED_Green_On();
	AX_Delayms(500);
	AX_LED_Green_Off();
	AX_Delayms(500);
	
	//LED点亮0.5S
	AX_LED_Red_On();
	AX_Delayms(500);
	AX_LED_Red_Off();
	AX_Delayms(500);
	
	//蜂鸣器长鸣0.5S
	AX_BEEP_On();
	AX_Delayms(500);
	AX_BEEP_Off();
	AX_Delayms(1000);
	
	AX_LED_Red_On();
	
	while (1)
	{	
       //调试串口输出信息		
		printf("Printf输出测试：%d \r\n",i);
		i++;
		AX_Delayms(100);
		
		//LED反转
	    AX_LED_Green_Toggle();
	    AX_LED_Red_Toggle();
	}
}



///******************************************************************************
//例程名称：VIN输入电压检测
//例程说明：串口循环输出采集到的电压值
//*******************************************************************************/
//int main(void)
//{
//	uint16_t vol;
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 
//
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  //LED初始化
//	
//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//VIN检测初始化
//	AX_VIN_Init();
//	
//	//提示信息
//	printf("*循环采集VIN输入口电压值\r\n");
//	
//	while (1) 
//	{	
//		//每100MS输出一次电池电压值
//		vol = AX_VIN_GetVol_X100();
//		printf("*VIN电压：%d(0.01V)\r\n",vol );	
//		AX_Delayms(100);
//		AX_LED_Green_Toggle();
//	}
//}


///******************************************************************************
//例程名称：KEY按键检测检测
//例程说明：按键按下后，LED灯闪烁一次
//*******************************************************************************/
//int main(void)
//{
//	uint8_t temp;
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    

//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 

//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  
//	
//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//初始化
//	AX_KEY_Init();
//	
//	while (1) 
//	{	
//		temp = AX_KEY_Scan();
//		
//		if(temp == 1)
//		{
//			AX_LED_Red_On();  
//			AX_Delayms(100);
//			AX_LED_Red_Off();
//		}
//		AX_Delayms(10);
//	}
//}	

///******************************************************************************
//例程名称：读取MPU6050数据并输出
//例程说明：读取MPU6050数据，通过串口Printf输出
//操作说明：串口输出三轴加速度，三轴陀螺仪数据，转动控制器，观察数据变化
//         使用我们的X-PrintfScope软件，可以观察到波形数据
//*******************************************************************************/
//int main(void)
//{
//	int16_t ax_acc[3],ax_gyro[3]; //IMU数据
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
//
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  //LED初始化

//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//MPU6050初始化  
//	AX_MPU6050_Init();    
//	AX_MPU6050_SetAccRange(AX_ACC_RANGE_2G);    //设置加速度量程
//	AX_MPU6050_SetGyroRange(AX_GYRO_RANGE_2000); //设置陀螺仪量程
//	AX_MPU6050_SetGyroSmplRate(200);            //设置陀螺仪采样率
//	AX_MPU6050_SetDLPF(AX_DLPF_ACC94_GYRO98);   //设置低通滤波器带宽
//	
//	while (1)
//	{	
//		//更新姿态、陀螺仪、加速度数据
//		AX_MPU6050_GetAccData(ax_acc);  //读取三轴加速度数据
//		AX_MPU6050_GetGyroData(ax_gyro);  //读取三轴陀螺仪数据		
//		
//		//调试串口输出信息
//		printf("@%d %d %d %d %d %d \r\n",
//		ax_acc[0],ax_acc[1],ax_acc[2],ax_gyro[0],ax_gyro[1],ax_gyro[2]);		
//		
//		AX_Delayms(50);
//	}
//}


///******************************************************************************
//例程名称：舵机控制例程
//例程说明：控制舵机-60度、0度、60度间隔运动
//操作说明：可在舵机接口中任意一路插入舵机，舵机即可循环运动，
//         如果插入多路舵机，或舵机负载扭矩大，请注意电源供电能力
//*******************************************************************************/
//int main(void)
//{
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 
//	
//	//初始化
//	AX_SERVO_S1234_Init();
//	AX_SERVO_S56_Init();
//	
//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//			
//	//软件延时初始化
//	AX_DELAY_Init();
//    AX_LED_Init();
//	
//	while (1) 
//	{		
//		printf("*-60度...... \r\n");		
//		AX_SERVO_S1_SetAngle(-600);
//		AX_SERVO_S2_SetAngle(-600);
//		AX_SERVO_S3_SetAngle(-600);
//		AX_SERVO_S4_SetAngle(-600);
//		AX_SERVO_S5_SetAngle(-600);
//		AX_SERVO_S6_SetAngle(-600);
//		AX_Delayms(1000);
//		
//		printf("*0度...... \r\n");
//		AX_SERVO_S1_SetAngle(0);
//		AX_SERVO_S2_SetAngle(0); 
//		AX_SERVO_S3_SetAngle(0);
//		AX_SERVO_S4_SetAngle(0);
//		AX_SERVO_S5_SetAngle(0);
//		AX_SERVO_S6_SetAngle(0);
//		AX_Delayms(1000);
//		
//		printf("*60度...... \r\n");
//		AX_SERVO_S1_SetAngle(600);
//		AX_SERVO_S2_SetAngle(600);
//		AX_SERVO_S3_SetAngle(600);
//		AX_SERVO_S4_SetAngle(600);
//		AX_SERVO_S5_SetAngle(600);
//		AX_SERVO_S6_SetAngle(600);
//		AX_Delayms(1000);
//	}
//}	


///******************************************************************************
//例程名称：直流电机PWM调速控制
//例程说明：控制2路电机变速正传和变速反转交替运行
//操作说明：电机可连接至二路控制接口中的任意一路，可以看到电机做间隔的增速正转和反转
//*******************************************************************************/
//int main(void)
//{
//	uint16_t temp;
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  

//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 
//		
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  
//	
//	//初始化
//	AX_MOTOR_Init();  //设置电机控制PWM频率为20K 

//	while (1) 
//	{	
//		//控制电机转动
//		for(temp=0; temp<=3600; temp++)
//		{
//			AX_MOTOR_A_SetSpeed(temp); 
//			AX_MOTOR_B_SetSpeed(temp);  			
//			AX_Delayms(2);
//		}
//		AX_MOTOR_A_SetSpeed(0); 
//		AX_MOTOR_B_SetSpeed(0);					
//		AX_Delayms(1000);
//		
//		//控制电机反向转动
//		for(temp=0; temp<=3600; temp++)
//		{
//			AX_MOTOR_A_SetSpeed(-temp); 
//			AX_MOTOR_B_SetSpeed(-temp); 			
//			AX_Delayms(2);
//		}
//		AX_MOTOR_A_SetSpeed(0); 
//		AX_MOTOR_B_SetSpeed(0); 					
//		AX_Delayms(1000);
//	}
//}	


///******************************************************************************
//例程名称：正交AB编码器例程
//例程说明：300ms采样编码器数值并串口输出显示，
//操作说明：编码器均可用，连接编码器后，即可手动转动电机观察串口输出值变化
//*******************************************************************************/
//int main(void)
//{		
//	//设置中断优先级分组
//	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//	
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 		
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//    AX_LED_Init();  
//	
//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG

// 	//正交编码器初始化
//    AX_ENCODER_A_Init();  
//	AX_ENCODER_B_Init();  

//	//提示信息
//	printf("正交编码器测试\r\n");

//	while (1) 
//	{		
//		printf("*A:%5d B:%5d  \r\n",AX_ENCODER_A_GetCounter(), AX_ENCODER_B_GetCounter());  
//		AX_Delayms(300);
//	}
//}	


///******************************************************************************
//例程名称：UART2串口通信例程
//例程说明：此例程不方便连接USB转串口操作，可以将发送接收引脚短接，实现自发，自收操作
//         通信采用X-Protocol，可使用塔克串口调试助手调试，具体详见视频教程
//*******************************************************************************/
//int main(void)
//{	
//	int16_t tmp1=1;
//	int16_t tmp2=1000;
//	uint8_t comdata[32];
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);    
//
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  

//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//串口初始化
//    AX_UART2_Init(115200);
//	
//	while (1)
//	{			
//		//组织数据，通过串口发送数据
//		tmp1 = tmp1 + 1;
//		comdata[0] = (u8)( tmp1 >> 8 );  
//		comdata[1] = (u8)( tmp1 );
//		
//		tmp2 = tmp2 - 1;
//		comdata[2] = (u8)( tmp2 >> 8 );
//		comdata[3] = (u8)( tmp2 );
//	 
//		AX_UART2_SendPacket(comdata, 4, 0x03);
//		
//		
//		//检测是否接收到数据，并通过调试串口输出前4个数据
//		if(AX_UART2_GetData(comdata))
//		{
//			printf("ID:%d  Data:%d %d %d %d \r\n",comdata[0],comdata[1],comdata[2],comdata[3],comdata[4]);		
//		}
//		
//		AX_Delayms(100);	
//	}
//}


///******************************************************************************
//例程名称：UART3串口通信例程
//例程说明：此例程不方便连接USB转串口操作，可以将发送接收引脚短接，实现自发，自收操作
//         通信采用X-Protocol，可使用塔克串口调试助手调试，具体详见视频教程
//*******************************************************************************/
//int main(void)
//{	
//	int16_t tmp1=1;
//	int16_t tmp2=1000;
//	uint8_t comdata[32];
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  

//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//串口初始化
//    AX_UART3_Init(115200);
//	
//	while (1)
//	{			
//		//组织数据，通过串口发送数据
//		tmp1 = tmp1 + 1;
//		comdata[0] = (u8)( tmp1 >> 8 );  
//		comdata[1] = (u8)( tmp1 );
//		
//		tmp2 = tmp2 - 1;
//		comdata[2] = (u8)( tmp2 >> 8 );
//		comdata[3] = (u8)( tmp2 );
//	 
//		AX_UART3_SendPacket(comdata, 4, 0x03);
//		
//		
//		//检测是否接收到数据，并通过调试串口输出前4个数据
//		if(AX_UART3_GetData(comdata))
//		{
//			printf("ID:%d  Data:%d %d %d %d \r\n",comdata[0],comdata[1],comdata[2],comdata[3],comdata[4]);		
//		}
//		
//		AX_Delayms(100);	
//	}
//}


///******************************************************************************
//例程名称：UART4串口通信例程
//例程说明：此例程不方便连接USB转串口操作，可以将发送接收引脚短接，实现自发，自收操作
//         通信采用X-Protocol，可使用塔克串口调试助手调试，具体详见视频教程
//*******************************************************************************/
//int main(void)
//{	
//	int16_t tmp1=1;
//	int16_t tmp2=1000;
//	uint8_t comdata[32];
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
//
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  

//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//串口初始化
//    AX_UART4_Init(115200);
//	
//	while (1)
//	{			
//		//组织数据，通过串口发送数据
//		tmp1 = tmp1 + 1;
//		comdata[0] = (u8)( tmp1 >> 8 );  
//		comdata[1] = (u8)( tmp1 );
//		
//		tmp2 = tmp2 - 1;
//		comdata[2] = (u8)( tmp2 >> 8 );
//		comdata[3] = (u8)( tmp2 );
//	 
//		AX_UART4_SendPacket(comdata, 4, 0x03);
//		
//		
//		//检测是否接收到数据，并通过调试串口输出前4个数据
//		if(AX_UART4_GetData(comdata))
//		{
//			printf("ID:%d  Data:%d %d %d %d \r\n",comdata[0],comdata[1],comdata[2],comdata[3],comdata[4]);		
//		}
//		
//		AX_Delayms(100);	
//	}
//}


///******************************************************************************
//例程名称：UART5串口通信例程
//例程说明：此例程不方便连接USB转串口操作，可以将发送接收引脚短接，实现自发，自收操作
//         通信采用X-Protocol，可使用塔克串口调试助手调试，具体详见视频教程
//*******************************************************************************/
//int main(void)
//{	
//	int16_t tmp1=1;
//	int16_t tmp2=1000;
//	uint8_t comdata[32];
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   
//
//	//JTAG口设置
//	AX_JTAG_Set(JTAG_SWD_DISABLE);  //关闭JTAG接口 
//	AX_JTAG_Set(SWD_ENABLE);  //打开SWD接口 
//	
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//	AX_LED_Init();  

//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n"); //输出空格，CPUBUG
//	
//	//串口初始化
//    AX_UART5_Init(115200);
//	
//	while (1)
//	{			
//		//组织数据，通过串口发送数据
//		tmp1 = tmp1 + 1;
//		comdata[0] = (u8)( tmp1 >> 8 );  
//		comdata[1] = (u8)( tmp1 );
//		
//		tmp2 = tmp2 - 1;
//		comdata[2] = (u8)( tmp2 >> 8 );
//		comdata[3] = (u8)( tmp2 );
//	 
//		AX_UART5_SendPacket(comdata, 4, 0x03);
//		
//		
//		//检测是否接收到数据，并通过调试串口输出前4个数据
//		if(AX_UART5_GetData(comdata))
//		{
//			printf("ID:%d  Data:%d %d %d %d \r\n",comdata[0],comdata[1],comdata[2],comdata[3],comdata[4]);		
//		}
//		
//		AX_Delayms(100);	
//	}
//}


///******************************************************************************
//例程名称：FLASH存储例程
//例程说明：实现片内FLASH的数据写入和读取，掉电参数存储
//*******************************************************************************/

//int main(void)
//{	
//	uint16_t tmp[3]= {0,100,1000};
//	
//	//设置中断优先级分组
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  
//		
//	//软件延时初始化
//	AX_DELAY_Init(); 	
//    AX_LED_Init();  
//	
//	//调试串口初始化
//	AX_UART1_Init(115200); //调试串口
//	printf("  \r\n");      //输出空格，CPUBUG	
//	
//	AX_Delayms(500);
//	
//	printf(" FLASH读写验证例程，写入指定数据，并读出后验证\r\n");
//	printf(" FLASH具有一定读写寿命，此程序不要频繁运行\r\n\r\n");
//	
//	//数组赋值
//	AX_FLASH_Write(0x10, tmp, 3);
//	printf(" 写入：%d %d %d \r\n",tmp[0],tmp[1],tmp[2]);
//	
//	//清除数据
//	tmp[0]=0;tmp[1]=0;tmp[2]=0;
//	
//	AX_FLASH_Read(0x10, tmp, 3);
//	printf(" 读出：%d %d %d \r\n",tmp[0],tmp[1],tmp[2]);	
//	
//	
//	printf(" 请比较读出和写入数据是否一致\r\n\r\n");
//	
//	while (1)
//	{
//		//发送数据
//		AX_LED_Green_On();
//		AX_Delayms(30);
//		AX_LED_Green_Off();
//		AX_Delayms(20);	
//	}
//	
//}


/******************* (C) 版权 2022 XTARK **************************************/

