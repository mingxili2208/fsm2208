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
  * @内  容  机器人运动学解析
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ax_kinematics.h"
#include "ax_robot.h"
#include "ax_speed.h"

//根据机器人类型，选择编译机器人运动学处理函数


#if (ROBOT_TYPE == ROBOT_MEC)
/**
  * @简  述  机器人运动学处理-麦克纳姆轮
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_Kinematics(void)
{

	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float)-((int16_t)AX_ENCODER_A_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float) ((int16_t)AX_ENCODER_B_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);		
	R_Wheel_C.RT = (float)-((int16_t)AX_ENCODER_C_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_C_SetCounter(0);
	R_Wheel_D.RT = (float) ((int16_t)AX_ENCODER_D_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_D_SetCounter(0);
	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = (( R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT + R_Wheel_D.RT)/4)*1000;
	R_Vel.RT_IY = ((-R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT - R_Wheel_D.RT)/4)*1000;
	R_Vel.RT_IW = ((-R_Wheel_A.RT + R_Wheel_B.RT - R_Wheel_C.RT + R_Wheel_D.RT)/4/(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2))*1000;	
	
	//机器人目标速度限制
	if( R_Vel.TG_IX > R_VX_LIMIT )    R_Vel.TG_IX = R_VX_LIMIT;
	if( R_Vel.TG_IX < (-R_VX_LIMIT))  R_Vel.TG_IX = (-R_VX_LIMIT);
	if( R_Vel.TG_IY > R_VY_LIMIT)     R_Vel.TG_IY = R_VY_LIMIT;
	if( R_Vel.TG_IY < (-R_VY_LIMIT))  R_Vel.TG_IY = (-R_VY_LIMIT);
	if( R_Vel.TG_IW > R_VW_LIMIT)     R_Vel.TG_IW = R_VW_LIMIT;
	if( R_Vel.TG_IW < (-R_VW_LIMIT))  R_Vel.TG_IW = (-R_VW_LIMIT);
	
	//目标速度转化为浮点类型
	R_Vel.TG_FX = R_Vel.TG_IX/1000.0;
	R_Vel.TG_FY = R_Vel.TG_IY/1000.0;
	R_Vel.TG_FW = R_Vel.TG_IW/1000.0;
	
	//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	R_Wheel_A.TG = R_Vel.TG_FX - R_Vel.TG_FY - R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	R_Wheel_B.TG = R_Vel.TG_FX + R_Vel.TG_FY + R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	R_Wheel_C.TG = R_Vel.TG_FX + R_Vel.TG_FY - R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	R_Wheel_D.TG = R_Vel.TG_FX - R_Vel.TG_FY + R_Vel.TG_FW*(MEC_WHEEL_BASE/2+MEC_ACLE_BASE/2);
	
	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);  
	R_Wheel_C.PWM = AX_SPEED_PidCtlC(R_Wheel_C.TG, R_Wheel_C.RT);  
	R_Wheel_D.PWM = AX_SPEED_PidCtlD(R_Wheel_D.TG, R_Wheel_D.RT);  
	
	//设置电机PWM值
	AX_MOTOR_A_SetSpeed( R_Wheel_A.PWM);
	AX_MOTOR_B_SetSpeed( R_Wheel_B.PWM);  
	AX_MOTOR_C_SetSpeed(-R_Wheel_C.PWM);
	AX_MOTOR_D_SetSpeed(-R_Wheel_D.PWM); 
	
	//printf("A%f B%f C%f D%f \r\n ",MOTOR_A.Wheel_RT, MOTOR_B.Wheel_RT, MOTOR_C.Wheel_RT, MOTOR_D.Wheel_RT  );
	//printf("A%d B%d C%d  \r\n ",R_Vel.I_X, R_Vel.I_Y, R_Vel.I_W );
}
#endif


#if (ROBOT_TYPE == ROBOT_FWD)
/**
  * @简  述  机器人运动学处理-四轮差速
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_Kinematics(void)
{
	
	//四轮差速由于轮胎摩擦力较大，采用60减速比电机，
	//注意，60减速比电机转向与30减速比刚好反向

	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float) ((int16_t)AX_ENCODER_A_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float)-((int16_t)AX_ENCODER_B_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);		
	R_Wheel_C.RT = (float) ((int16_t)AX_ENCODER_C_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_C_SetCounter(0);
	R_Wheel_D.RT = (float)-((int16_t)AX_ENCODER_D_GetCounter()*FWD_WHEEL_SCALE);
	AX_ENCODER_D_SetCounter(0);
	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = (( R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT + R_Wheel_D.RT)/4)*1000;
	R_Vel.RT_IY = 0;
	R_Vel.RT_IW = ((-R_Wheel_A.RT + R_Wheel_B.RT - R_Wheel_C.RT + R_Wheel_D.RT)/(2*FWD_WB_SCALE*FWD_WHEEL_BASE))*1000;	
	
	//机器人目标速度限制
	if( R_Vel.TG_IX > R_VX_LIMIT )    R_Vel.TG_IX = R_VX_LIMIT;
	if( R_Vel.TG_IX < (-R_VX_LIMIT))  R_Vel.TG_IX = (-R_VX_LIMIT);
	if( R_Vel.TG_IY > R_VY_LIMIT)     R_Vel.TG_IY = R_VY_LIMIT;
	if( R_Vel.TG_IY < (-R_VY_LIMIT))  R_Vel.TG_IY = (-R_VY_LIMIT);
	if( R_Vel.TG_IW > R_VW_LIMIT)     R_Vel.TG_IW = R_VW_LIMIT;
	if( R_Vel.TG_IW < (-R_VW_LIMIT))  R_Vel.TG_IW = (-R_VW_LIMIT);
	
	//目标速度转化为浮点类型
	R_Vel.TG_FX = R_Vel.TG_IX/1000.0;
	R_Vel.TG_FY = 0;
	R_Vel.TG_FW = R_Vel.TG_IW/1000.0;
	
	//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	R_Wheel_A.TG = R_Vel.TG_FX - R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	R_Wheel_B.TG = R_Vel.TG_FX + R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	R_Wheel_C.TG = R_Vel.TG_FX - R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	R_Wheel_D.TG = R_Vel.TG_FX + R_Vel.TG_FW*(FWD_WHEEL_BASE*FWD_WB_SCALE/2);
	
	
	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);  
	R_Wheel_C.PWM = AX_SPEED_PidCtlC(R_Wheel_C.TG, R_Wheel_C.RT);  
	R_Wheel_D.PWM = AX_SPEED_PidCtlD(R_Wheel_D.TG, R_Wheel_D.RT);  
	
	if(R_Vel.TG_IX ==0 && R_Vel.TG_IW ==0)
	{
		R_Wheel_A.PWM = 0;   
		R_Wheel_B.PWM = 0;  
		R_Wheel_C.PWM = 0;  
		R_Wheel_D.PWM = 0;  	
	}
	
	//设置电机PWM值
	AX_MOTOR_A_SetSpeed( -R_Wheel_A.PWM);
	AX_MOTOR_B_SetSpeed( -R_Wheel_B.PWM);  
	AX_MOTOR_C_SetSpeed(  R_Wheel_C.PWM);
	AX_MOTOR_D_SetSpeed(  R_Wheel_D.PWM); 
	
	//输出调试信息
	//printf("@%f  %f  %f %f  \r\n",R_Wheel_A.RT,R_Wheel_B.RT, R_Wheel_C.RT ,R_Wheel_D.RT);
	//printf("A%d B%d C%d  \r\n ",R_Vel.I_X, R_Vel.I_Y, R_Vel.I_W );
}
#endif


#if (ROBOT_TYPE == ROBOT_AKM)
/**
  * @简  述  机器人运动学处理-阿克曼
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_Kinematics(void)
{

	float akm_spd;  //速度
    float akm_angle; //前轮转向角度
	float akm_angle_r;  //右前轮转向角度
	float akm_angle_s;  //舵机转向角度
	float radius;  //转弯半径
	int16_t servo_angle; //舵机控制角度
	
	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float)-((int16_t)AX_ENCODER_A_GetCounter()*AKM_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float) ((int16_t)AX_ENCODER_B_GetCounter()*AKM_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);	
	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	/*******正向运动学解析(根据前进速度和舵机转角计算)*********************************/
	
	//机器人目标速度限制
	if( R_Vel.TG_IX > R_VX_LIMIT )    R_Vel.TG_IX = R_VX_LIMIT;
	if( R_Vel.TG_IX < (-R_VX_LIMIT))  R_Vel.TG_IX = (-R_VX_LIMIT);
	
	//转向角度限制
//	if( ax_akm_angle > R_VX_LIMIT )    ax_akm_angle = R_VX_LIMIT;
//	if( ax_akm_angle < (-R_VX_LIMIT))  ax_akm_angle = (-R_VX_LIMIT);
	
	//转换为浮点数计算
	akm_spd = R_Vel.TG_IX/1000.0;
	akm_angle = ax_akm_angle/1000.0;
	
	//根据转向角度计算转弯半径
	radius = AKM_ACLE_BASE/tan(akm_angle);		
	
	//根据前轮角度计算右前轮角度
	if(ax_akm_angle !=0 )
	{

		//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
		R_Wheel_A.TG = akm_spd*(radius-0.5*AKM_WHEEL_BASE)/radius;
		R_Wheel_B.TG = akm_spd*(radius+0.5*AKM_WHEEL_BASE)/radius;					
		
		//计算右前轮角度
		akm_angle_r = (atan(AKM_ACLE_BASE/((AKM_ACLE_BASE/tan(akm_angle*0.01745))+0.5*AKM_WHEEL_BASE)))*(180.0/PI);
	}
	else
	{
		//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
		R_Wheel_A.TG = akm_spd;
		R_Wheel_B.TG = akm_spd;	
		
		//计算右前轮角度
		akm_angle_r = 0;
	}	
	
    //根据右前轮角度，计算舵机转向角度
	akm_angle_s = 	-(0.0041*akm_angle_r*akm_angle_r + 1.2053*akm_angle_r)*180/PI;
	
	//根据舵机转向角度，计算舵机PWM控制量
	servo_angle = (akm_angle_s*10 + ax_akm_offset);  
	
	//设置舵机角度，S1,S2任意一个都可以
	AX_SERVO_S1_SetAngle(servo_angle);
	AX_SERVO_S2_SetAngle(servo_angle);	
	
	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);   
		
	//设置电机PWM值
	AX_MOTOR_A_SetSpeed(R_Wheel_A.PWM);
	AX_MOTOR_B_SetSpeed(R_Wheel_B.PWM);  
	AX_MOTOR_C_SetSpeed(0);
	AX_MOTOR_D_SetSpeed(0);	
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = ((R_Wheel_A.RT + R_Wheel_B.RT)/2)*1000;
	R_Vel.RT_IY = 0;
	R_Vel.RT_IW = ((-R_Wheel_A.RT + R_Wheel_B.RT)/AKM_WHEEL_BASE)*1000;	
}
#endif


#if (ROBOT_TYPE == ROBOT_TWD)
/**
  * @简  述  机器人运动学处理-两轮差速
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_Kinematics(void)
{
	
	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float)-((int16_t)AX_ENCODER_A_GetCounter()*TWD_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float) ((int16_t)AX_ENCODER_B_GetCounter()*TWD_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);			
	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = ((R_Wheel_A.RT + R_Wheel_B.RT)/2)*1000;
	R_Vel.RT_IY = 0;
	R_Vel.RT_IW = ((-R_Wheel_A.RT + R_Wheel_B.RT)/TWD_WHEEL_BASE)*1000;		
	
	//机器人目标速度限制
	if( R_Vel.TG_IX > R_VX_LIMIT )    R_Vel.TG_IX = R_VX_LIMIT;
	if( R_Vel.TG_IX < (-R_VX_LIMIT))  R_Vel.TG_IX = (-R_VX_LIMIT);
	if( R_Vel.TG_IY > R_VY_LIMIT)     R_Vel.TG_IY = R_VY_LIMIT;
	if( R_Vel.TG_IY < (-R_VY_LIMIT))  R_Vel.TG_IY = (-R_VY_LIMIT);
	if( R_Vel.TG_IW > R_VW_LIMIT)     R_Vel.TG_IW = R_VW_LIMIT;
	if( R_Vel.TG_IW < (-R_VW_LIMIT))  R_Vel.TG_IW = (-R_VW_LIMIT);
	
	//目标速度转化为浮点类型
	R_Vel.TG_FX = R_Vel.TG_IX/1000.0;
	R_Vel.TG_FY = 0;
	R_Vel.TG_FW = R_Vel.TG_IW/1000.0;
	
	//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	R_Wheel_A.TG = R_Vel.TG_FX - R_Vel.TG_FW*(TWD_WHEEL_BASE/2);
	R_Wheel_B.TG = R_Vel.TG_FX + R_Vel.TG_FW*(TWD_WHEEL_BASE/2);	
	

	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);  
 
	//设置电机PWM值
	AX_MOTOR_A_SetSpeed( R_Wheel_A.PWM);
	AX_MOTOR_B_SetSpeed( R_Wheel_B.PWM); 
	AX_MOTOR_C_SetSpeed(0);
	AX_MOTOR_D_SetSpeed(0);	
 
	
	//printf("A%f B%f  \r\n ",MOTOR_A.Wheel_RT, MOTOR_B.Wheel_RT  );
	//printf("A%d B%d C%d  \r\n ",R_Vel.I_X, R_Vel.I_Y, R_Vel.I_W );	
	
}
#endif


#if (ROBOT_TYPE == ROBOT_TNK)
/**
  * @简  述  机器人运动学处理-履带式
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_Kinematics(void)
{
	
	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float)-((int16_t)AX_ENCODER_A_GetCounter()*TNK_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float) ((int16_t)AX_ENCODER_B_GetCounter()*TNK_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);			
	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = ((R_Wheel_A.RT + R_Wheel_B.RT)/2)*1000;
	R_Vel.RT_IY = 0;
	R_Vel.RT_IW = ((-R_Wheel_A.RT + R_Wheel_B.RT)/TNK_WHEEL_BASE)*1000;		
	
	//机器人目标速度限制
	if( R_Vel.TG_IX > R_VX_LIMIT )    R_Vel.TG_IX = R_VX_LIMIT;
	if( R_Vel.TG_IX < (-R_VX_LIMIT))  R_Vel.TG_IX = (-R_VX_LIMIT);
	if( R_Vel.TG_IY > R_VY_LIMIT)     R_Vel.TG_IY = R_VY_LIMIT;
	if( R_Vel.TG_IY < (-R_VY_LIMIT))  R_Vel.TG_IY = (-R_VY_LIMIT);
	if( R_Vel.TG_IW > R_VW_LIMIT)     R_Vel.TG_IW = R_VW_LIMIT;
	if( R_Vel.TG_IW < (-R_VW_LIMIT))  R_Vel.TG_IW = (-R_VW_LIMIT);
	
	//目标速度转化为浮点类型
	R_Vel.TG_FX = R_Vel.TG_IX/1000.0;
	R_Vel.TG_FY = 0;
	R_Vel.TG_FW = R_Vel.TG_IW/1000.0;
	
	//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	R_Wheel_A.TG = R_Vel.TG_FX - R_Vel.TG_FW*(TNK_WHEEL_BASE/2);
	R_Wheel_B.TG = R_Vel.TG_FX + R_Vel.TG_FW*(TNK_WHEEL_BASE/2);	

	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);  
 
	//设置电机PWM值
	AX_MOTOR_A_SetSpeed( R_Wheel_A.PWM);
	AX_MOTOR_B_SetSpeed( R_Wheel_B.PWM); 
	AX_MOTOR_C_SetSpeed(0);
	AX_MOTOR_D_SetSpeed(0);	
	
}
#endif


#if (ROBOT_TYPE == ROBOT_OMT)
/**
  * @简  述  机器人运动学处理-三轮全向
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_Kinematics(void)
{
	
	//通过编码器获取车轮实时转速m/s
	R_Wheel_A.RT = (float)-((int16_t)AX_ENCODER_A_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_A_SetCounter(0);
	R_Wheel_B.RT = (float) ((int16_t)AX_ENCODER_B_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_B_SetCounter(0);		
	R_Wheel_C.RT = (float)-((int16_t)AX_ENCODER_C_GetCounter()*MEC_WHEEL_SCALE);
	AX_ENCODER_C_SetCounter(0);

	
	//调试输出轮子转速
	//printf("@%f  %f   \r\n",R_Wheel_A.RT,R_Wheel_B.RT);
	
	//运动学正解析，由机器人轮子速度计算机器人速度
	R_Vel.RT_IX = ((-R_Wheel_A.RT + R_Wheel_B.RT)/SQRT3)*1000;
	R_Vel.RT_IY = ((-R_Wheel_A.RT + R_Wheel_B.RT - 2.0*R_Wheel_C.RT)/3)*1000;
	R_Vel.RT_IW = (( R_Wheel_A.RT + R_Wheel_B.RT + R_Wheel_C.RT)/(3*OMT_WHEEL_RADIUS))*1000;	
	
	//机器人目标速度限制
	if( R_Vel.TG_IX > R_VX_LIMIT )    R_Vel.TG_IX = R_VX_LIMIT;
	if( R_Vel.TG_IX < (-R_VX_LIMIT))  R_Vel.TG_IX = (-R_VX_LIMIT);
	if( R_Vel.TG_IY > R_VY_LIMIT)     R_Vel.TG_IY = R_VY_LIMIT;
	if( R_Vel.TG_IY < (-R_VY_LIMIT))  R_Vel.TG_IY = (-R_VY_LIMIT);
	if( R_Vel.TG_IW > R_VW_LIMIT)     R_Vel.TG_IW = R_VW_LIMIT;
	if( R_Vel.TG_IW < (-R_VW_LIMIT))  R_Vel.TG_IW = (-R_VW_LIMIT);
	
	//目标速度转化为浮点类型
	R_Vel.TG_FX = R_Vel.TG_IX/1000.0;
	R_Vel.TG_FY = R_Vel.TG_IY/1000.0;
	R_Vel.TG_FW = R_Vel.TG_IW/1000.0;
	
	//运动学逆解析，由机器人目标速度计算电机轮子速度（m/s）
	R_Wheel_A.TG = -0.5*R_Vel.TG_FX + 0.5*R_Vel.TG_FY + OMT_WHEEL_RADIUS*R_Vel.TG_FW;
	R_Wheel_B.TG =  0.5*R_Vel.TG_FX + 0.5*R_Vel.TG_FY + OMT_WHEEL_RADIUS*R_Vel.TG_FW;
	R_Wheel_C.TG = -R_Vel.TG_FY + OMT_WHEEL_RADIUS*R_Vel.TG_FW;

	
	//利用PID算法计算电机PWM值
	R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);   
	R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);  
	R_Wheel_C.PWM = AX_SPEED_PidCtlC(R_Wheel_C.TG, R_Wheel_C.RT);  
 
	
	//设置电机PWM值
	AX_MOTOR_A_SetSpeed( R_Wheel_A.PWM);
	AX_MOTOR_B_SetSpeed( R_Wheel_B.PWM);  
	AX_MOTOR_C_SetSpeed(-R_Wheel_C.PWM);
	AX_MOTOR_D_SetSpeed(0); 
	
	//printf("A%f B%f C%f D%f \r\n ",MOTOR_A.Wheel_RT, MOTOR_B.Wheel_RT, MOTOR_C.Wheel_RT, MOTOR_D.Wheel_RT  );
	//printf("A%d B%d C%d  \r\n ",R_Vel.I_X, R_Vel.I_Y, R_Vel.I_W );
}
#endif

/**
  * @简  述  机器人停止运动状态
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_Stop(void)
{
	
	//设置电机PWM值
	AX_MOTOR_A_SetSpeed(0);
	AX_MOTOR_B_SetSpeed(0);  
	AX_MOTOR_C_SetSpeed(0);
	AX_MOTOR_D_SetSpeed(0);  
	
}


/******************* (C) 版权 2023 XTARK **************************************/

