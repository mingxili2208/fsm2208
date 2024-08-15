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
  * @内  容  电机PWM控制函数
  *
  ******************************************************************************
  * @说  明
  *
  ******************************************************************************
  */

#include "ax_motor.h" 

/**
  * @简  述  电机PWM控制初始化，PWM频率为20KHZ	
  * @参  数  无
  * @返回值  无
  */
void AX_MOTOR_AB_Init(void)
{ 
	
    GPIO_InitTypeDef GPIO_InitStructure; 

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
	//时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	

	//复用功能配置
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1); 
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1); 
	
	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	//定时器配置
	TIM_TimeBaseStructure.TIM_Period=4200-1;   //自动重装载值	
	TIM_TimeBaseStructure.TIM_Prescaler=1;     //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    //PWM1 Mode configuration: Channel1 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	    //占空比初始化
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel2
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel3
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel4
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    //使能定时器
    TIM_Cmd(TIM1, ENABLE);   
	
	//使能MOE位
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
}

/**
  * @简  述 电机PWM速度控制
  * @参  数 speed 电机转速数值，范围-4200~4200
  * @返回值 无
  */
void AX_MOTOR_A_SetSpeed(int16_t speed)
{
	int16_t temp;
	
	temp = speed;
	
	if(temp>4200)
		temp = 4200;
	if(temp<-4200)
		temp = -4200;
	
	if(temp > 0)
	{
		TIM_SetCompare1(TIM1, 4200);
		TIM_SetCompare2(TIM1, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM1, 4200);
		TIM_SetCompare1(TIM1, (4200 + temp));
	}
}


/**
  * @简  述 电机PWM速度控制
  * @参  数 speed 电机转速数值，范围-4200~4200
  * @返回值 无
  */
void AX_MOTOR_B_SetSpeed(int16_t speed)
{
	int16_t temp;
	
	temp = speed;
	
	if(temp>4200)
		temp = 4200;
	if(temp<-4200)
		temp = -4200;
	
	if(temp > 0)
	{
		TIM_SetCompare3(TIM1, 4200);
		TIM_SetCompare4(TIM1, (4200 - temp));
	}
	else
	{
		TIM_SetCompare4(TIM1, 4200);
		TIM_SetCompare3(TIM1, (4200 + temp));
	}
}


/**
  * @简  述  电机PWM控制初始化，PWM频率为20KHZ	
  * @参  数  无
  * @返回值  无
  */
void AX_MOTOR_CD_Init(void)
{ 
	
    GPIO_InitTypeDef GPIO_InitStructure; 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
	//时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);	

	//复用功能配置
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource5,GPIO_AF_TIM9);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource6,GPIO_AF_TIM9);
	
	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	//定时器配置 
	TIM_TimeBaseStructure.TIM_Period=4200-1;   //自动重装载值	
	TIM_TimeBaseStructure.TIM_Prescaler=1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);

    //PWM1 Mode configuration: Channel1 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	    //占空比初始化
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel2
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM9, ENABLE);

    //使能定时器
    TIM_Cmd(TIM9, ENABLE);   
	
	
	//时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);		
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);	

	//复用功能配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12);/*复用*/
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM12);/*复用*/
	

	//配置IO口为复用功能-定时器通道
	GPIO_InitStructure.GPIO_Pin    =  GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode   =  GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed  =  GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType  =  GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd   =  GPIO_PuPd_UP;        //上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	//定时器配置
	TIM_TimeBaseStructure.TIM_Period=4200-1;   //自动重装载值	
	TIM_TimeBaseStructure.TIM_Prescaler=0;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);

    //PWM1 Mode configuration: Channel1 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;	    //占空比初始化
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	
    TIM_OC1Init(TIM12, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);

    //PWM1 Mode configuration: Channel2
    TIM_OC2Init(TIM12, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM12, ENABLE);	

    //使能定时器
    TIM_Cmd(TIM12, ENABLE);   
}

/**
  * @简  述 电机PWM速度控制
  * @参  数 speed 电机转速数值，范围-4200~4200
  * @返回值 无
  */
void AX_MOTOR_C_SetSpeed(int16_t speed)
{
	int16_t temp;
	
	temp = speed;
	
	if(temp>4200)
		temp = 4200;
	if(temp<-4200)
		temp = -4200;
	
	if(temp > 0)
	{
		TIM_SetCompare1(TIM9, 4200);
		TIM_SetCompare2(TIM9, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM9, 4200);
		TIM_SetCompare1(TIM9, (4200 + temp));
	}
	
}


/**
  * @简  述 电机PWM速度控制
  * @参  数 speed 电机转速数值，范围-4200~4200
  * @返回值 无
  */
void AX_MOTOR_D_SetSpeed(int16_t speed)
{
	int16_t temp;
	
	temp = speed;
	
	if(temp>4200)
		temp = 4200;
	if(temp<-4200)
		temp = -4200;
	
	if(temp > 0)
	{
		TIM_SetCompare1(TIM12, 4200);
		TIM_SetCompare2(TIM12, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM12, 4200);
		TIM_SetCompare1(TIM12, (4200 + temp));
	}
}


/******************* (C) 版权 2023 XTARK **************************************/
