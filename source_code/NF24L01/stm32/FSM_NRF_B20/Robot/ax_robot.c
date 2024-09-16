/**			                                                    
		   ____                    _____ _______ _____       @塔克创新
		  / __ \                  / ____|__   __|  __ \ 
		 | |  | |_ __   ___ _ __ | |       | |  | |__) |
		 | |  | | '_ \ / _ \ '_ \| |       | |  |  _  / 
		 | |__| | |_) |  __/ | | | |____   | |  | | \ \ 
		  \____/| .__/ \___|_| |_|\_____|  |_|  |_|  \_\
				| |                                     
				|_|                OpenCTR   机器人控制器
									 
  ****************************************************************************** 
  *           
  * 版权所有： @塔克创新  版权所有，盗版必究
  * 公司网站： www.xtark.cn   www.tarkbot.com
  * 淘宝店铺： https://xtark.taobao.com  
  * 塔克微信： 塔克创新（关注公众号，获取最新更新资讯）
  *           
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @内  容  机器人控制主函数
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_robot.h"
#include "ax_light.h"
#include "ax_kinematics.h"
#include "ax_control.h"
#include "24l01.h"

//机器人速度数据
ROBOT_Velocity  R_Vel;

//机器人轮子数据
ROBOT_Wheel  R_Wheel_A,R_Wheel_B;

//机器人IMU数据
ROBOT_Imu  R_Imu;

//机器人RGB数据
ROBOT_Light  R_Light;

//机器人速度数据
ROBOT_Velocity  R_Vel;

//机器人电池电压数据
uint16_t R_Bat_Vol;  

//IMU数据
int16_t ax_imu_acc_data[3];  
int16_t ax_imu_gyro_data[3]; 
int16_t ax_imu_gyro_offset[3]; 

//电机PID控制参数
int16_t ax_motor_kp=700;      
int16_t ax_motor_kd=900; 

//机器人运动使能开关,默认打开状态
uint8_t ax_robot_move_enable = 1;

//灯光开关失能，默认打开状态
uint8_t ax_light_enable = 1;

//蜂鸣器鸣叫一声
uint8_t ax_beep_ring = 0;

//PS2手柄键值结构体
JOYSTICK_TypeDef my_joystick;  

/**
 * @brief NRF24L01L info_structures;
 * @param uint8_t ST:state of remote (0 closed, 1 open)
 * @param uint8_t steering_anglespeed: change steering
 * @param uint8_t steering_angle_velocity: 	speed of angle change
 * @param uint8_t speed: speed of x direction
 * @param uint8_t acceleration: speedup
 * 
 */
NRF_CTL_INFO nrt_ctl_info;

//控制方式选择
uint8_t ax_control_mode = CTL_ROS;


//阿克曼机器人前轮转向角度
int16_t ax_akm_angle = 0;

//舵机云台关节角度
int16_t ax_joint_angle[2] = {0};  


//函数定义
void JOINT_Control(void);       //舵机云台处理
void ROBOT_IMUHandle(void);     //IMU数据处理
void ROBOT_SendDataToRos(void);  //发送数据
	
	
/**
  * @简  述  机器人管理任务
  * @参  数  无
  * @返回值  无
  */
void Robot_Task(void* parameter)
{	

	//用于保存上次时间。调用后系统自动更新
	static portTickType PreviousWakeTime;

	//设置延时时间20ms，将时间转为节拍数 
	const portTickType TimeIncrement = pdMS_TO_TICKS(20);
	
	//获取当前系统时间 
	PreviousWakeTime = xTaskGetTickCount();
	
	while(1)
	{
		
		//调用绝对延时函数20ms,执行频率50HZ
		vTaskDelayUntil(&PreviousWakeTime, TimeIncrement );
		
		//控制方式选择
		if(ax_control_mode != 0)
		{
			if      (ax_control_mode == CTL_PS2)    AX_CTL_Ps2();    //PS2手柄控制
			else if (ax_control_mode == CTL_APP)    AX_CTL_App();    //APP控制
			else if (ax_control_mode == CTL_RMS)    AX_CTL_RemoteSbus();   //SBUS航模遥控器控制
			else if (ax_control_mode == CTL_NRF) 	FSM_CTL_NRF();   //NRF24L01航模??控器控制
		}
		
		//机器人运动学处理
		AX_ROBOT_Kinematics();	
		
		//舵机云台控制
		JOINT_Control();		
		
		//获取PMU6050加速度数据
        ROBOT_IMUHandle();
		
		//数据发送
		ROBOT_SendDataToRos();
			
	}
}

/**
  * @简  述  舵机云台控制函数
  * @参  数  无
  * @返回值  无
  */
void JOINT_Control(void)
{
	//幅度限制保护
	if(ax_joint_angle[0] > JOINTA_UP_LIMIT)	   ax_joint_angle[0] = JOINTA_UP_LIMIT;
	if(ax_joint_angle[0] < JOINTA_LOW_LIMIT)   ax_joint_angle[0] = JOINTA_LOW_LIMIT;
	
	if(ax_joint_angle[1] > JOINTB_UP_LIMIT)	   ax_joint_angle[1] = JOINTB_UP_LIMIT;
	if(ax_joint_angle[1] < JOINTB_LOW_LIMIT)   ax_joint_angle[1] = JOINTB_LOW_LIMIT;
	
	//查看舵机角度
	//printf("A%d  \r\n ", ax_joint_angle[0] );
	
	//设置舵机角度
	AX_SERVO_S1_SetAngle( ax_joint_angle[0] + JOINTA_ANGLE_OFFSET);
	AX_SERVO_S2_SetAngle( -ax_joint_angle[1] - JOINTB_ANGLE_OFFSET);	
		
}

/**
  * @简  述  机器人IMU数据处理
  * @参  数  无
  * @返回值  无
  */
void ROBOT_IMUHandle(void)
{
		
	//获取PMU6050加速度数据
	AX_MPU6050_GetAccData(ax_imu_acc_data);
	
	//IMU坐标向机器人ROS坐标变换
	R_Imu.ACC_X =  ax_imu_acc_data[1];  //ROS坐标X轴对应IMU的Y轴
	R_Imu.ACC_Y = -ax_imu_acc_data[0];  //ROS坐标Y轴对应IMU的X轴反向
	R_Imu.ACC_Z =  ax_imu_acc_data[2];  //ROS坐标Z轴对应IMU的Z轴
	
	//获取PMU6050陀螺仪数据
	AX_MPU6050_GetGyroData(ax_imu_gyro_data);
	
	//陀螺仪加入零票校准数据
	ax_imu_gyro_data[0] += ax_imu_gyro_offset[0];
	ax_imu_gyro_data[1] += ax_imu_gyro_offset[1];
	ax_imu_gyro_data[2] += ax_imu_gyro_offset[2];
	
	//IMU坐标向机器人ROS坐标变换
	R_Imu.GYRO_X =  ax_imu_gyro_data[1];  //ROS坐标X轴对应IMU的Y轴
	R_Imu.GYRO_Y = -ax_imu_gyro_data[0];  //ROS坐标Y轴对应IMU的X轴反向
	R_Imu.GYRO_Z =  ax_imu_gyro_data[2];  //ROS坐标Z轴对应IMU的Z轴
	
	//观察数据
    //printf("@ %d  \r\n",R_Imu.ACC_X);
}

/**
  * @简  述  机器人发送数据到ROS
  * @参  数  无
  * @返回值  无
  */
void ROBOT_SendDataToRos(void)
{
    //串口发送数据
	static uint8_t comdata[20]; 	

	//加速度 = (ax_acc/32768) * 2G  
	comdata[0] = (u8)( R_Imu.ACC_X >> 8 );  
	comdata[1] = (u8)( R_Imu.ACC_X );
	comdata[2] = (u8)( R_Imu.ACC_Y >> 8 );
	comdata[3] = (u8)( R_Imu.ACC_Y );
	comdata[4] = (u8)( R_Imu.ACC_Z >> 8 );
	comdata[5] = (u8)( R_Imu.ACC_Z );
	
	//陀螺仪角速度 = (ax_gyro/32768) * 500
	comdata[6] = (u8)( R_Imu.GYRO_X >> 8 );
	comdata[7] = (u8)( R_Imu.GYRO_X );
	comdata[8] = (u8)( R_Imu.GYRO_Y >> 8 );
	comdata[9] = (u8)( R_Imu.GYRO_Y );
	comdata[10] = (u8)( R_Imu.GYRO_Z	>> 8 );
	comdata[11] = (u8)( R_Imu.GYRO_Z );
	
	//机器人速度值 单位为m/s，放大1000倍
	comdata[12] = (u8)( R_Vel.RT_IX >> 8 );
	comdata[13] = (u8)( R_Vel.RT_IX );
	comdata[14] = (u8)( R_Vel.RT_IY >> 8 );
	comdata[15] = (u8)( R_Vel.RT_IY );
	comdata[16] = (u8)( R_Vel.RT_IW >> 8 );
	comdata[17] = (u8)( R_Vel.RT_IW );
	
	//电池电压
	comdata[18] = (u8)( R_Bat_Vol >> 8 );
	comdata[19] = (u8)( R_Bat_Vol );

	//TTL串口发送数据
    AX_UART4_SendPacket(comdata, 20, ID_UTX_DATA);		
	
	//USB串口发送数据（与调试串口共用，需要调试时可注释掉该句）
    AX_UART1_SendPacket(comdata, 20, ID_UTX_DATA);	
}


/**
  * @简  述  琐事管理任务
  * @参  数  无
  * @返回值  无
  */
void Trivia_Task(void* parameter)
{	
	//计数变量
	static uint16_t ax_bat_vol_cnt = 0; 
	
	while (1)
	{	
		
		/*****电池管理***********************************/
		
		//采集电池电压
	    R_Bat_Vol = AX_VIN_GetVol_X100();
		
		//调试输出电池电压数据
        //printf("@ %d  \r\n",R_Bat_Vol);		
		
		//电量低于40%
		if(R_Bat_Vol < VBAT_40P)  
		{
			//红灯开始闪烁警示
			AX_LED_Red_Toggle();
			
			//电量低于20%
			if(R_Bat_Vol < VBAT_20P)
			{
				//红灯常亮
				AX_LED_Red_On();
				
				//电量低于10%，关闭系统进入保护状态
				if(R_Bat_Vol < VBAT_10P) 
				{
					//低压时间计数
					ax_bat_vol_cnt++;
					
					//超过10次，进入关闭状态
					if(ax_bat_vol_cnt > 10 )
					{
						//关闭绿灯，红灯常亮
						AX_LED_Green_Off();
						AX_LED_Red_On();
						
						//任务挂起
						vTaskSuspend(Robot_Task_Handle);
						vTaskSuspend(Disp_Task_Handle);
						
						//电机速度设置为0
						AX_MOTOR_A_SetSpeed(0);
						AX_MOTOR_B_SetSpeed(0);  
						
						//清除OLED启动画面显示
						AX_OLED_ClearScreen();  //
						
						//蜂鸣器鸣叫报警
						while(1)
						{	
							//显示机器人停止信息
							AX_OLED_DispStr(0, 3, "     Low power      ", 0);	
							AX_OLED_DispStr(0, 5, "  Robot has stopped ", 0);	
							AX_BEEP_On();
							vTaskDelay(30);
							AX_BEEP_Off();
							
							vTaskDelay(1000);	
							AX_OLED_ClearScreen(); 
							vTaskDelay(1000);						
						}								
					}
				}
				else
				{
					ax_bat_vol_cnt = 0;
				}				
			}
		}
		else
		{
			//红灯关闭
			AX_LED_Red_Off();
		}			
		/*****led 管理************************************** */
		if(NRF_led_flag!=3)
		{
			if(NRF_led_flag==1)
			{
				AX_LED_Green_On();
				vTaskDelay(100); 
				AX_LED_Green_Off();
				
				//鸣叫一声标志复位
				NRF_led_flag = 3;
			}
			else
			{
				AX_LED_Red_On();
				vTaskDelay(100); 
				AX_LED_Red_Off();
				//红灯短亮
				NRF_led_flag = 3;		
			}
		}
		/*****蜂鸣器鸣叫管理***********************************/
		if(NRF_beep_flag!=3)
		{
			if(NRF_beep_flag==1)
			{
				AX_BEEP_On();
				vTaskDelay(100); 
				AX_BEEP_Off();
				
				//鸣叫一声标志复位
				NRF_beep_flag = 3;
			}
			else
			{
				AX_BEEP_On();
				vTaskDelay(200); 
				AX_BEEP_Off();
				vTaskDelay(100);
				AX_BEEP_On();
				vTaskDelay(200); 
				AX_BEEP_Off();
				//短鸣叫两声标志复位
				NRF_beep_flag = 3;		
			}
		}
		if(ax_beep_ring != 0)
		{
			if(ax_beep_ring == BEEP_SHORT)
			{
				//短鸣叫一声
				AX_BEEP_On();
				vTaskDelay(200); 
				AX_BEEP_Off();
				
				//鸣叫一声标志复位
				ax_beep_ring = 0;
			}
			else //
			{
				//长鸣叫一声
				AX_BEEP_On();
				vTaskDelay(1000); 
				AX_BEEP_Off();
				
				//鸣叫一声标志复位
				ax_beep_ring = 0;				
			}
		}
		
		//LED系统心跳指示
		AX_LED_Green_Toggle();	
		
		
        //循环周期500ms
		vTaskDelay(500); 
	}			
}

/**
  * @简  述  按键处理任务
  * @参  数  无
  * @返回值  无
  */
void Key_Task(void* parameter)
{	
	uint8_t  i;
	//int16_t  temp;
	
	while (1)
	{		
		//按键扫描
		if(AX_KEY_Scan() != 0)
		{
			//软件延时
			vTaskDelay(50);  
			
			//确定按键按下
			if(AX_KEY_Scan() != 0)
			{
				//等待按键抬起
				for(i=0; i<200; i++)
				{

					vTaskDelay(50);
					
					if(AX_KEY_Scan() == 0)
					{
						break;
					}
					
					//按键时长3S时，给出提示音
					if(i == 60)
					{
						AX_BEEP_On();
						vTaskDelay(200);
						AX_BEEP_Off();
					}
				}

				//短按检测,小于1S
				if(i < 20)
				{
					//灯光效果切换
					if(R_Light.M < LEFFECT6)
						R_Light.M++;
					else
						R_Light.M = LEFFECT1;
				}
				
				//中按检测,大于3S，小于10S
				if(i>60 && i<200)
				{
					//暂无
				}				
				
					
				//长按检测，大于10S
				if(i == 200)
				{
					//暂无					
				}
			}
		}
		
		//循环周期
		vTaskDelay(50);    
	}			
}


/**
  * @简  述  灯光处理任务
  * @参  数  无
  * @返回值  无
  */
void Light_Task(void* parameter)
{	

	while (1)
	{	
        //延时30ms，30HZ显示频率		
		vTaskDelay(30); 
		
		//判断前灯是否打开
		if(ax_light_enable != 0)
		{
			//正常显示
			AX_LIGHT_Show();
		}
		else
		{
			//关闭状态
			AX_RGB_SetFullColor(0x00, 0x00, 0x00);
		}
	}			
}


/**
  * @简  述  按键处理任务
  * @参  数  无
  * @返回值  无
  */
void Disp_Task(void* parameter)
{	

	while (1)
	{	
		//延时	
		vTaskDelay(100); 
		
		//显示控制模式
		if(ax_control_mode)
		{
			if      (ax_control_mode == CTL_PS2)    AX_OLED_DispStr(90, 2, "PS2", 0);   //PS2手柄控制
			else if (ax_control_mode == CTL_APP)    AX_OLED_DispStr(90, 2, "APP", 0);   //APP控制
			else if (ax_control_mode == CTL_RMS)    AX_OLED_DispStr(90, 2, "RMS", 0);   //SBUS航模遥控器控制			
		}
		else
		{
			//ROS控制模式
			AX_OLED_DispStr(90, 2, "ROS", 0);
		}
		
		//显示电池电压，陀螺仪Z轴数据,
		AX_OLED_DispValue(30, 3, (R_Bat_Vol*0.1), 2, 1, 0);
		AX_OLED_DispValue(90, 3, (R_Imu.GYRO_Z ), 6, 0, 0);	
		
		vTaskDelay(100); 
		//显示轮子实时速度,舵机偏角
		AX_OLED_DispValue(30, 5, (ax_akm_angle*10 ), 2, 3, 0);
		AX_OLED_DispValue(96, 5, (R_Vel.RT_IX ), 5, 2, 0);	
		AX_OLED_DispValue(30, 6, (R_Wheel_A.RT*100), 2, 2, 0);
		AX_OLED_DispValue(90, 6, (R_Wheel_B.RT*100), 2, 2, 0);	
	}			
}

/**
  * @简  述  PS2数据获取任务
  * @参  数  无
  * @返回值  无
  */
void Ps2_Task(void* parameter)
{	

	//用于保存上次时间。调用后系统自动更新
	static portTickType PreviousWakeTime1;

	//设置延时时间20ms，将时间转为节拍数 
	const portTickType TimeIncrement1 = pdMS_TO_TICKS(20);
	
	//获取当前系统时间 
	PreviousWakeTime1 = xTaskGetTickCount();
	
	while(1)
	{
		
		//调用绝对延时函数20ms,执行频率50HZ
		vTaskDelayUntil(&PreviousWakeTime1, TimeIncrement1 );
		
		//读取PS2手柄键值
		AX_PS2_ScanKey(&my_joystick);
		
		//不在PS2控制模式下
		if(ax_control_mode != CTL_PS2)
		{
			//判断是否开启PS2手柄控制
			//START按键被按下后，左边摇杆上推，进入PS2控制模式
			if((my_joystick.btn1 == PS2_BT1_START) && (my_joystick.LJoy_UD == 0x00))
			{
				//切换到PS2模式
				ax_control_mode = CTL_PS2;	

				//执行蜂鸣器鸣叫提示
				ax_beep_ring = BEEP_SHORT;
			}
		}
		
//		//打印手柄键值
//		printf("MODE:%2x BTN1:%2x BTN2:%2x RJOY_LR:%2x RJOY_UD:%2x LJOY_LR:%2x LJOY_UD:%2x\r\n",
//		my_joystick.mode, my_joystick.btn1, my_joystick.btn2, 
//		my_joystick.RJoy_LR, my_joystick.RJoy_UD, my_joystick.LJoy_LR, my_joystick.LJoy_UD);
	}
}

/******************* (C) 版权 2023 XTARK **************************************/

/**
  * @brief   NRF controller
  * @param 	 NULL
  * @return  NULL
  */
void Nrf_Task(void* parameter)
{	

//		while(1)
//	{
//		if(NRF24L01_Check())
//		{
//			AX_BEEP_On();
//			AX_Delayms(20);	
//			AX_BEEP_Off();
//			AX_Delayms(200);
//			
//		}else	break;
//	}
	//用于保存上次时间。调用后系统自动更新
	static portTickType PreviousWakeTime1;

	//设置延时时间20ms，将时间转为节拍数 
	const portTickType TimeIncrement1 = pdMS_TO_TICKS(20);
	
	//获取当前系统时间 
	PreviousWakeTime1 = xTaskGetTickCount();
	
	while(1)
	{
		
		//调用绝对延时函数20ms,执行频率50HZ
		vTaskDelayUntil(&PreviousWakeTime1, TimeIncrement1 );
		
		//读取PS2手柄键值
		//AX_PS2_ScanKey(&my_joystick);
		FSM_NRF_ScanKey(&nrt_ctl_info);
		
		//不在PS2控制模式下
		if(ax_control_mode != CTL_NRF)
		{
			//判断是否开启PS2手柄控制
			//START按键被按下后，左边摇杆上推，进入PS2控制模式
			if((nrt_ctl_info.ST == 0x01) )
			{
				//切换到PS2模式
				ax_control_mode = CTL_NRF;	

				//执行蜂鸣器鸣叫提示
				ax_beep_ring = BEEP_SHORT;
			}
		}
		
	}
}
